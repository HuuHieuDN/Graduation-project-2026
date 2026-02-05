// dashboard.js (fixed)

const NODE_API_BASE = "https://presidiary-affirmatively-simone.ngrok-free.dev"; // sửa IP máy chạy node

async function setAntiTheft(enabled) {
  try {
    const res = await fetch(`${NODE_API_BASE}/api/antitheft`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled })
    });

    // nếu server lỗi thì ném ra để thấy trong console
    if (!res.ok) {
      const text = await res.text();
      throw new Error(`HTTP ${res.status}: ${text}`);
    }

    const json = await res.json();
    console.log("API /api/antitheft:", json);

    // hiển thị ngay trên UI (không cần đợi DB update)
    const antiBadge = document.getElementById("antitheft-badge");
    if (antiBadge) {
      antiBadge.textContent = enabled ? "ĐANG BẬT" : "ĐANG TẮT";
      antiBadge.className = "badge " + (enabled ? "badge-warn" : "badge-neutral");
    }
  } catch (e) {
    console.error("AntiTheft API error:", e);
    alert("Gửi lệnh chống trộm thất bại. Kiểm tra NODE_API_BASE/IP và server.js đang chạy.");
  }
}

function wireButtons() {
  const btnOn = document.getElementById("btn-antitheft-on");
  const btnOff = document.getElementById("btn-antitheft-off");

  if (btnOn) btnOn.addEventListener("click", () => setAntiTheft(true));
  if (btnOff) btnOff.addEventListener("click", () => setAntiTheft(false));

  if (!btnOn || !btnOff) {
    console.warn("Không tìm thấy nút antitheft. Kiểm tra id btn-antitheft-on/off trong dashboard.html");
  }
}

async function loadLatest() {
  try {
    const res = await fetch("lastest.php");
    if (!res.ok) return;

    const json = await res.json();
    if (!json.ok) return;

    const d = json.data || {};

    // Device ID + last update
    const devEl = document.getElementById("device-id");
    if (devEl) devEl.textContent = d.deviceId || "--";

    const updEl = document.getElementById("last-update");
    if (updEl) updEl.textContent = d.lastUpdate ? new Date(d.lastUpdate).toLocaleString() : "--";

    // Accident/status
    const statusBadge = document.getElementById("device-status-badge");
    if (statusBadge) {
      let statusText = "Không rõ";
      if (d.status === "NONE") statusText = "AN TOÀN";
else if (d.status === "CRASH") statusText = "TAI NẠN";
else if (d.status === "FALL") statusText = "NGÃ XE";
else statusText = d.status;


      statusBadge.textContent = statusText;
      statusBadge.className = "badge " + (d.status === "NONE" ? "badge-safe" : "badge-danger");
    }

    // Anti-theft (DB)
    const antiBadge = document.getElementById("antitheft-badge");
    if (antiBadge) {
      antiBadge.textContent = d.antiTheft ? "ĐANG BẬT" : "ĐANG TẮT";
      antiBadge.className = "badge " + (d.antiTheft ? "badge-warn" : "badge-neutral");
    }

    // Sensors
    const accEl = document.getElementById("acc-total");
if (accEl) accEl.textContent = (Number(d?.mpu?.accTotal || 0)).toFixed(2) + " g";

    const angleYEl = document.getElementById("angle-y");
if (angleYEl) angleYEl.textContent = (Number(d?.mpu?.angleY || 0)).toFixed(1) + "°";

    const angleZEl = document.getElementById("angle-z");
    if (angleZEl) angleZEl.textContent = "0.0°";

   const simEl = document.getElementById("sim-status");
if (simEl) {
  const simReady = !!d?.sim?.simReady;
  const moduleOk = !!d?.sim?.moduleOk;
  const csq = d?.sim?.csq;

  if (!moduleOk) simEl.textContent = "LỖI MODULE";
  else if (!simReady) simEl.textContent = "CHƯA SẴN SÀNG";
  else simEl.textContent = `SẴN SÀNG`;
}

   const satsEl = document.getElementById("gps-sats");
if (satsEl) satsEl.textContent = d?.gps?.sats ?? 0;
  } catch (e) {
    console.error("loadLatest error:", e);
  }
}

// init
// wireButtons();
loadLatest();
setInterval(loadLatest, 3000);
