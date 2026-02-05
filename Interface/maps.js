let map, marker;

function initMap(lat, lng) {
  map = L.map("map").setView([lat, lng], 17);
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
  }).addTo(map);
  marker = L.marker([lat, lng]).addTo(map);
  marker.bindPopup("Vị trí xe máy").openPopup();
}

function updateMap(lat, lng) {
  if (!map) {
    initMap(lat, lng);
  } else {
    const ll = [lat, lng];
    marker.setLatLng(ll);
    map.setView(ll);
  }
}

async function loadPosition() {
  try {
    const res = await fetch("lastest.php");
    const json = await res.json();
    if (!json.ok) return;
    const g = json.data.gps;
    if (!g.valid) return;

    document.getElementById("gps-lat").textContent = g.lat.toFixed(6);
    document.getElementById("gps-lng").textContent = g.lng.toFixed(6);
    updateMap(g.lat, g.lng);
  } catch (e) {
    console.error(e);
  }
}

loadPosition();
setInterval(loadPosition, 5000);
