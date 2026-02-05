<?php
header('Content-Type: application/json; charset=utf-8');

$mysqli = new mysqli("localhost", "root", "", "moto");
if ($mysqli->connect_errno) {
  http_response_code(500);
  echo json_encode(['ok' => false, 'error' => 'DB_ERROR']);
  exit;
}

// Lấy các event đã được server.js ghi theo "đổi trạng thái"
$sql = "SELECT id, deviceId, gpsOk, lat, lng, status, antiTheft, accTotal, angleY, sats, created_at
        FROM history_events
        WHERE status IN ('CRASH','FALL','LOST1','LOST2','NONE')
        ORDER BY id DESC
        LIMIT 200";

$res = $mysqli->query($sql);

$events = [];
if ($res) {
  while ($row = $res->fetch_assoc()) {

    $status = strtoupper($row['status']);
    $type = 'Không rõ';
    $note = '';

    if ($status === 'CRASH') {
      $type = 'Tai nạn';
      $note = 'Va chạm, gia tốc: ' . $row['accTotal'];
    } elseif ($status === 'FALL') {
      $type = 'Tai nạn';
      $note = 'Xe ngã/nghiêng bất thường.';
    } elseif ($status === 'LOST1' || $status === 'LOST2') {
      $type = 'Mất trộm';
      $note = 'Nghi ngờ bị di chuyển (' . $status . ').';
    } elseif ($status === 'NONE') {
      $type = 'Bình thường';
      $note = 'Thiết bị đã trở lại trạng thái bình thường.';
    }

    $events[] = [
      'id' => (int)$row['id'],
      'time' => $row['created_at'],
      'type' => $type,
      'lat' => (float)$row['lat'],
      'lng' => (float)$row['lng'],
      'note' => $note,
      'deviceId' => $row['deviceId'],
    ];
  }
}

echo json_encode(['ok' => true, 'events' => $events]);
