'use strict';
// One in-flight request: errors without request_id cannot race another request.
// Changes are coalesced, never reconnect the socket, and never retry forever.
class SubscriptionControl {
  constructor(send, selected, message, apply) {
    Object.assign(this, { send, selected, message, apply });
    this.streaming = true;
    this.revision = 0;
    this.request = 0;
    this.pending = null;
    this.timer = null;
    this.online = false;
  }

  cancel() {
    clearTimeout(this.timer);
    clearTimeout(this.pending?.timer);
    this.timer = null;
    this.pending = null;
  }

  connected() {
    this.cancel();
    this.online = true;
    ++this.revision;
    this.flush();
  }

  disconnected() {
    this.online = false;
    this.cancel();
  }

  changed() {
    ++this.revision;
    clearTimeout(this.timer);
    this.timer = null;
    if (!this.streaming) {
      this.message('配信停止中：選択内容は「配信開始」で送信します');
      return;
    }
    this.message('変更待ち…（200 msでまとめて送信）');
    this.timer = setTimeout(() => {
      this.timer = null;
      this.flush();
    }, 200);
  }

  start(streaming = true) {
    this.streaming = streaming;
    ++this.revision;
    clearTimeout(this.timer);
    this.timer = null;
    this.message(streaming ? '配信開始待ち…' : '配信停止待ち…');
    this.flush();
  }

  fail(reason) {
    this.cancel();
    this.streaming = false;
    this.message(`配信設定失敗: ${reason}。自動再試行なし`);
  }

  flush() {
    if (this.pending) return;
    if (!this.online) {
      this.message('配信設定未送信：未接続。再接続してください');
      return;
    }
    const fields = this.selected();
    const empty = fields.length === 0;
    if (empty) this.streaming = false;
    const request = {
      id: ++this.request, revision: this.revision,
      streaming: this.streaming, empty, fields
    };
    this.pending = request;
    this.message(empty ? '全項目解除：配信停止の応答待ち…' : '配信応答待ち…');
    request.timer = setTimeout(() => {
      if (this.pending === request) this.fail('応答期限超過／受理状態不明');
    }, 2500);
    const packet = request.streaming
      ? { type: 'subscribe', request_id: request.id, period_ms: 50, fields }
      : { type: 'stop', request_id: request.id };
    try { this.send(JSON.stringify(packet)); }
    catch { this.fail('送信できませんでした'); }
  }

  receive(response) {
    const request = this.pending;
    if (!request) return;
    if (response.type === 'error') {
      this.fail(response.reason || '設定拒否');
      return;
    }
    if (response.type !== 'subscribed' || response.request_id !== request.id) return;
    clearTimeout(request.timer);
    this.pending = null;
    // A selection made while waiting supersedes this ACK, including its graph.
    if (request.revision !== this.revision) {
      if (!this.timer) this.flush();
      return;
    }
    if (response.streaming !== request.streaming || response.period_ms !== 50 ||
      !Array.isArray(response.fields) ||
      response.fields.length !== (request.streaming ? request.fields.length : 0) ||
      (request.streaming && request.fields.some(key => !response.fields.includes(key)))) {
      this.fail('受理内容が要求と一致しません');
      return;
    }
    this.apply(response, request.empty);
    this.message(`受理：世代 ${response.subscription_id} / 50 ms / ` +
      `${response.streaming ? '配信中' : '配信停止'} / 選択${response.fields.length}項目` +
      (request.empty ? '（全項目解除。再開は「配信開始」）' : ''));
  }
}
