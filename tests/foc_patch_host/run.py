"""Python-only LF/CRLF/locale/idempotence tests; also runnable on Windows."""
from pathlib import Path
from contextlib import redirect_stdout
import hashlib
import importlib.util
import io
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch as mock
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('foc_patch', 'scripts/patch_foc_observation.py')
patcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patcher)
raw = Path(patcher.DEFAULT_TARGET).read_bytes().replace(b'\r\n', b'\n')
original = raw.decode('utf-8')
if hashlib.sha256(raw).hexdigest() == patcher.PATCHED_SHA256:
    for before, after in reversed(patcher.CHANGES):
        original = original.replace(after, before, 1)
assert hashlib.sha256(original.encode('utf-8')).hexdigest() == patcher.ORIGINAL_SHA256
patched = original
for before, after in patcher.CHANGES:
    patched = patched.replace(before, after, 1)
assert hashlib.sha256(patched.encode('utf-8')).hexdigest() == patcher.PATCHED_SHA256


class PatchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='foc-patch-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'Arduino-FOC'
        self.path = self.root / 'src/common/base_classes/FOCMotor.cpp'
        self.path.parent.mkdir(parents=True)
        (self.root / 'library.properties').write_bytes(
            b'name=Simple FOC\nversion=2.4.0\nauthor=DO_NOT_PRINT_AUTHOR\npassword=DO_NOT_PRINT_SECRET\n')
        (self.root / '.git').mkdir()
        (self.root / '.git/HEAD').write_bytes(patcher.EXPECTED_COMMIT.encode('ascii'))
        (self.root / '.git/config').write_bytes(b'DO_NOT_READ_REMOTE_OR_CREDENTIALS')

    def reject(self, raw, reason):
        self.path.write_bytes(raw)
        stamp = self.path.stat().st_mtime_ns
        output = io.StringIO()
        with redirect_stdout(output), self.assertRaisesRegex(RuntimeError, 'refusing unverified'):
            patcher.patch(self.path)
        self.assertEqual(self.path.read_bytes(), raw)
        self.assertEqual(self.path.stat().st_mtime_ns, stamp)
        text = output.getvalue()
        self.assertIn('rejected_condition=' + reason, text)
        self.assertIn('declared_version=2.4.0', text)
        self.assertIn('raw_sha256=' + hashlib.sha256(raw).hexdigest(), text)
        self.assertIn('line_endings=', text)
        self.assertIn('original_sha256_match=', text)
        self.assertIn('patched_sha256_match=', text)
        for secret in ['DO_NOT_PRINT_AUTHOR', 'DO_NOT_PRINT_SECRET', 'DO_NOT_READ_REMOTE_OR_CREDENTIALS',
                       'PRIVATE_SOURCE_MARKER', str(self.root)]:
            self.assertNotIn(secret, text)
        return text

    def test_original_and_applied_lf_crlf_preserve_bytes_on_reapply(self):
        for eol in [b'\n', b'\r\n']:
            for source, applied in [(original, False), (patched, True)]:
                with self.subTest(eol=eol, applied=applied):
                    raw = source.encode('utf-8').replace(b'\n', eol)
                    expected = patched.encode('utf-8').replace(b'\n', eol)
                    self.path.write_bytes(raw)
                    info = patcher.inspect_source(self.path)
                    self.assertEqual(info['state'], 'PATCHED' if applied else 'ORIGINAL')
                    self.assertEqual(info['line_endings'], 'LF' if eol == b'\n' else 'CRLF')
                    # Simulate a locale-sensitive Windows environment: text APIs
                    # must not be used by either source or metadata handling.
                    with mock.object(Path, 'read_text', side_effect=AssertionError('locale-sensitive read')):
                        with mock.object(Path, 'write_text', side_effect=AssertionError('newline-sensitive write')):
                            self.assertEqual(patcher.patch(self.path), not applied)
                            self.assertEqual(self.path.read_bytes(), expected)
                            stamp = self.path.stat().st_mtime_ns
                            for _ in range(3):
                                self.assertFalse(patcher.patch(self.path))
                                self.assertEqual(self.path.read_bytes(), expected)
                                self.assertEqual(self.path.stat().st_mtime_ns, stamp)
                    self.assertNotIn(b'\r\r\n', self.path.read_bytes())

    def test_legacy_windows_cp932_and_cp1252_hash_failure(self):
        raw = original.encode('utf-8')
        self.assertTrue(any(byte > 127 for byte in raw))
        for codepage in ['cp932', 'cp1252']:
            legacy = raw.decode(codepage).encode('utf-8')
            self.assertNotEqual(hashlib.sha256(legacy).hexdigest(), patcher.ORIGINAL_SHA256)
            # Do not "repair" source actually re-encoded through a wrong codepage.
            self.reject(legacy, 'source_hash:')
        # Original UTF-8 bytes still apply correctly regardless of locale.
        self.path.write_bytes(raw)
        self.assertTrue(patcher.patch(self.path))

    def test_unknown_source_even_when_all_anchors_match(self):
        output = self.reject((original + '\n// PRIVATE_SOURCE_MARKER\n').encode('utf-8'), 'source_hash:')
        self.assertIn('original_sha256_match=False patched_sha256_match=False', output)
        self.assertIn('anchor[1] original_count=1', output)

    def test_changed_algorithm_and_partial_patch_rejected(self):
        self.reject(original.replace('> 0.5f', '> 0.6f', 1).encode('utf-8'), 'source_hash:')
        before, after = patcher.CHANGES[0]
        self.reject(original.replace(before, after, 1).encode('utf-8'), 'source_hash:')
        self.reject((patched + '\n// PRIVATE_SOURCE_MARKER').encode('utf-8'), 'source_hash:')

    def test_mixed_cr_bom_invalid_utf8_rejected_without_writes(self):
        raw = original.encode('utf-8')
        self.assertIn('line_endings=MIXED', self.reject(raw.replace(b'\n', b'\r\n', 1), 'line_endings:'))
        self.assertIn('line_endings=CR', self.reject(raw.replace(b'\n', b'\r'), 'line_endings:'))
        self.assertIn('encoding=UTF-8-BOM', self.reject(b'\xef\xbb\xbf' + raw, 'encoding:'))
        self.assertIn('encoding=INVALID-UTF-8', self.reject(raw + b'\xff', 'encoding:'))

    def test_anchor_and_generated_hash_checks_still_fail_closed(self):
        raw = original.encode('utf-8')
        with mock.object(patcher, 'CHANGES', [('nonexistent anchor', 'replacement')]):
            self.reject(raw, 'anchor[1]:')
        with mock.object(patcher, 'PATCHED_SHA256', '0' * 64):
            self.reject(raw, 'output_hash:')

    def test_untrusted_metadata_never_printed(self):
        (self.root / 'library.properties').write_bytes(b'version=DO_NOT_PRINT_SECRET\nauthor=DO_NOT_PRINT_AUTHOR')
        (self.root / '.git/HEAD').write_bytes(b'ref: refs/heads/DO_NOT_PRINT_SECRET')
        self.path.write_bytes(original.encode('utf-8'))
        info = patcher.inspect_source(self.path)
        output = io.StringIO()
        with redirect_stdout(output):
            patcher.print_diagnostics(info)
        self.assertIn('declared_version=unknown', output.getvalue())
        self.assertIn('checkout_commit=unknown', output.getvalue())
        self.assertNotIn('DO_NOT_PRINT', output.getvalue())
        self.assertTrue(patcher.patch(self.path)) # Full known source hash is the authority.

    def test_cli_check_is_read_only_for_original_applied_and_unknown(self):
        script = Path('scripts/patch_foc_observation.py')
        for source, expected in [(original, 0), (patched, 0), (original + '\n// drift', 1)]:
            raw = source.encode('utf-8').replace(b'\n', b'\r\n')
            self.path.write_bytes(raw)
            stamp = self.path.stat().st_mtime_ns
            result = subprocess.run([sys.executable, str(script), '--check', str(self.path)],
                                    capture_output=True, check=False)
            self.assertEqual(result.returncode, expected, result.stderr)
            self.assertIn(b'line_endings=CRLF', result.stdout)
            self.assertEqual(self.path.read_bytes(), raw)
            self.assertEqual(self.path.stat().st_mtime_ns, stamp)
        result = subprocess.run([sys.executable, str(script), '--check', str(self.path.parent/'missing.cpp')],
                                capture_output=True, check=False)
        self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
