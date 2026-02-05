async function loadHistory() {
  try {
    const res = await fetch("history.php");
    const json = await res.json();
    if (!json.ok) return;

    const tbody = document.getElementById("history-body");
    tbody.innerHTML = "";

    json.events.forEach((e, idx) => {
      const tr = document.createElement("tr");
 const lat = e.lat;
const lng = e.lng;
const gmapsUrl = `https://www.google.com/maps/?q=${lat},${lng}`;

tr.innerHTML = `
  <td>${idx + 1}</td>
  <td>${new Date(e.time).toLocaleString()}</td>
  <td>${e.type}</td>
  <td>
    <a href="${gmapsUrl}" target="_blank" rel="noopener noreferrer">
      Xem trên Google Maps
    </a>
  </td>
  <td>${e.note}</td>
`;

      tbody.appendChild(tr);
    });
  } catch (err) {
    console.error(err);
  }
}

loadHistory();
setInterval(loadHistory, 5000);
