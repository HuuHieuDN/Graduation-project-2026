// dashboard_mqtt.js
const DEVICE_ID = "ESP32-MOTO-01";
const CMD_TOPIC = "/mcd/cmd/ESP32-MOTO-01";
const LOCATION_TOPIC = "/mcd/location/ESP32-MOTO-01";

// Nếu dashboard chạy https => dùng wss (khuyến nghị)
const client = mqtt.connect("wss://broker.emqx.io:8084/mqtt", {
  clientId: "web-" + DEVICE_ID + "-" + Math.random().toString(16).slice(2),
  keepalive: 60,
  clean: true,
});

client.on("connect", () => {
  console.log("MQTT WS connected");
  client.subscribe(LOCATION_TOPIC, { qos: 0 });
});

client.on("error", (e) => console.error("MQTT error:", e));

function sendAntiTheft(enabled) {
  const payload = JSON.stringify({
    deviceId: DEVICE_ID,
    needUpdateLocation: false,
    toggleAntiTheft: !!enabled,
    offWarning: false,
  });

  client.publish(CMD_TOPIC, payload, { qos: 1, retain: false });
  console.log("Published:", CMD_TOPIC, payload);
}

window.addEventListener("DOMContentLoaded", () => {
  document.getElementById("btn-antitheft-on")?.addEventListener("click", () => sendAntiTheft(true));
  document.getElementById("btn-antitheft-off")?.addEventListener("click", () => sendAntiTheft(false));
});

// (Tuỳ chọn) nghe trạng thái device để cập nhật UI
client.on("message", (topic, msgBuf) => {
  if (topic !== LOCATION_TOPIC) return;
  try {
    const d = JSON.parse(msgBuf.toString());
    const anti = !!d.antiTheft;

    const badge = document.getElementById("antitheft-badge");
    if (badge) badge.textContent = anti ? "ĐANG BẬT" : "ĐANG TẮT";
  } catch (e) {}
});
