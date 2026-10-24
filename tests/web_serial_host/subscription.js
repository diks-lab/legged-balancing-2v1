// Real subscription controller, deterministic clock and delayed/out-of-order ACKs.
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
let now = 0, id = 0;
const timers = new Map();
const context = vm.createContext({
  setTimeout(fn, delay) { timers.set(++id, { fn, at: now + delay }); return id; },
  clearTimeout(key) { timers.delete(key); }
});
vm.runInContext(fs.readFileSync('data/subscribe.js', 'utf8') +
  '\nthis.Controller = SubscriptionControl;', context);
function advance(ms) {
  const end = now + ms;
  for (;;) {
    const next = [...timers].sort((a, b) => a[1].at - b[1].at)[0];
    if (!next || next[1].at > end) break;
    now = next[1].at;
    timers.delete(next[0]);
    next[1].fn();
  }
  now = end;
}
let fields = ['angle_deg'], message = '', sent = [], applied = [];
const controller = new context.Controller(
  text => sent.push(JSON.parse(text)), () => [...fields],
  text => { message = text; }, (response, empty) => applied.push({ response, empty }));
function ack(packet = sent.at(-1), overrides = {}) {
  const response = { type: 'subscribed', request_id: packet.request_id,
    subscription_id: packet.request_id, period_ms: 50,
    fields: packet.fields || [], streaming: packet.type !== 'stop', ...overrides };
  controller.receive(response);
}
controller.connected();
assert.equal(sent.length, 1); ack();
assert(message.includes('受理') && controller.streaming);
fields.push('reference_deg'); controller.changed(); advance(100);
fields.push('gyro_deg_s'); controller.changed(); advance(199);
assert.equal(sent.length, 1); advance(1);
assert.equal(sent.length, 2);
const older = sent.at(-1);
fields.push('right_command_v'); controller.changed(); advance(200);
assert.equal(sent.length, 2); // One in flight, no extra queue.
ack(older);
assert.equal(applied.length, 1); // Superseded ACK never changes graph.
assert.equal(sent.length, 3);
const latest = sent.at(-1);
ack(older); assert.equal(applied.length, 1);
ack(latest); assert.equal(applied.length, 2);
assert.deepEqual(Array.from(applied.at(-1).response.fields), fields);
fields = []; controller.changed(); advance(200);
assert.equal(sent.at(-1).type, 'stop'); ack();
assert(!controller.streaming && applied.at(-1).empty);
const stoppedCount = sent.length;
fields = ['angle_deg']; controller.changed(); advance(5000);
assert.equal(sent.length, stoppedCount); // No restart on selection while stopped.
controller.start(); ack(); assert(controller.streaming);
controller.start(false); ack();
fields.push('reference_deg'); controller.changed(); advance(500);
assert.equal(sent.at(-1).type, 'stop');
controller.disconnected(); controller.connected();
assert.equal(sent.at(-1).type, 'stop'); ack(); // Reconnect preserves explicit stop.
controller.start(); const timedOut = sent.at(-1);
advance(2500); assert(message.includes('期限超過') && !controller.streaming);
const count = sent.length; advance(10000); assert.equal(sent.length, count);
controller.start(); ack(timedOut); assert(controller.pending); ack();
assert(controller.streaming);
controller.start(); controller.receive({ type: 'error', reason: 'invalid_subscription' });
assert(!controller.streaming && message.includes('invalid_subscription'));
advance(10000); assert.equal(timers.size, 0);
controller.start(); ack(sent.at(-1), { fields: ['unexpected'] });
assert(message.includes('一致しません') && !controller.streaming);
// Stop clicked while a start is waiting wins over the late start ACK.
controller.start(); const start = sent.at(-1);
controller.start(false); ack(start); assert.equal(sent.at(-1).type, 'stop'); ack();
assert(!controller.streaming && !controller.pending);
controller.disconnected(); const offlineCount = sent.length;
controller.start(); assert.equal(sent.length, offlineCount);
assert(message.includes('未接続'));
const throwing = new context.Controller(() => { throw Error('send failed'); },
  () => ['angle_deg'], text => { message = text; }, () => assert.fail('unexpected ACK'));
throwing.connected();
assert(message.includes('送信できません') && !throwing.pending);
advance(10000); assert.equal(timers.size, 0);
controller.connected(); ack(sent.at(-1), { fields: ['unexpected'] });
assert(message.includes('一致しません') && !controller.streaming);
console.log('PASS: debounce/single in-flight, superseded/old ACK, clear-all stop, stopped edits, explicit restart, stop priority, timeout/error/mismatch without retries, reconnect/offline');
