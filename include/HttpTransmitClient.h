#pragma once
// NetworkClient must already be declared. This adapter copies shared socket
// ownership, then drops ONLY its own RX reference. Never stop/close to free RX.
// Supported protected layout/lifetime is verified by check_http_client_core.py.
class HttpTransmitClient : public NetworkClient {
 public:
  explicit HttpTransmitClient(const NetworkClient& source):NetworkClient(source){_rxBuffer.reset();}
};
