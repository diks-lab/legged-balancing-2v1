'use strict';
class DiagnosticHistory {
  constructor() { this.clear(); }

  clear() {
    this.rows = [];
    this.clock = 0;
    this.raw = null;
    this.breakNext = false;
  }

  push(sample) {
    const previous = this.rows.at(-1);
    if (this.raw !== null) {
      const delta = (sample.sample_ms - this.raw) >>> 0;
      if (delta > 0x7fffffff) return;
      this.clock += delta;
    } else {
      this.clock = sample.sample_ms;
    }
    this.raw = sample.sample_ms;
    const gap = this.breakNext || Boolean(previous && (
      ((sample.seq - previous.seq) >>> 0) !== 1 ||
      sample.subscription_id !== previous.subscription_id ||
      sample.boot_id !== previous.boot_id ||
      sample.dropped !== previous.dropped ||
      this.clock - previous.time > 150
    ));
    this.breakNext = false;
    this.rows.push({ ...sample, time: this.clock, gap });
    while (this.rows.length > 1200 ||
      this.clock - this.rows[0].time > 60000) this.rows.shift();
  }

  csv() {
    const keys = SIGNALS.map(s => s[0]).filter(k =>
      this.rows.some(r => k in r.values));
    const quote = v => '"' + String(v ?? '').replaceAll('"', '""') + '"';
    const head = [
      'boot_id', 'subscription_id', 'seq', 'source_seq', 'sample_ms',
      'foc_capture_us', 'age_ms', 'dropped', 'stop_ms', 'stop_reason',
      'faults', 'running', 'disabled', 'ps4', 'foc_ready', ...keys
    ];
    return head.join(',') + '\n' + this.rows.map(s =>
      head.map(k => quote(k in s ? s[k] : s.values[k])).join(',')
    ).join('\n');
  }
}
if (typeof module !== 'undefined') module.exports = { DiagnosticHistory };
