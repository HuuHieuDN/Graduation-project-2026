<?php
// lastest.php - trả JSON bản ghi mới nhất từ bảng locations
header('Content-Type: application/json; charset=utf-8');

$mysqli = new mysqli('localhost', 'root', '', 'moto'); // sửa DB, user, pass cho đúng

if ($mysqli->connect_errno) {
  http_response_code(500);
  echo json_encode(['ok' => false, 'error' => 'DB_ERROR']);
  exit;
}

$sql = "SELECT deviceId, gpsOk, lat, lng, status, antiTheft, accTotal, angleY, sats, created_at
        FROM locations
        ORDER BY id DESC
        LIMIT 1";
$res = $mysqli->query($sql);

if (!$res || $res->num_rows == 0) {
  echo json_encode(['ok' => false, 'message' => 'NO_DATA']);
  exit;
}

$row = $res->fetch_assoc();

// map sang cấu trúc state cho app.js
$data = [
  'deviceId'   => $row['deviceId'],
  'status'     => $row['status'],
  'antiTheft'  => (bool)$row['antiTheft'],
  'lastUpdate' => $row['created_at'],
  'mpu' => [
    'ax'       => 0,
    'ay'       => 0,
    'az'       => 0,
    'accTotal' => (float)$row['accTotal'],
    'angleY'   => (float)$row['angleY'],
    'angleZ'   => 0,
  ],
  'gps' => [
    'valid' => (bool)$row['gpsOk'],
    'lat'   => (float)$row['lat'],
    'lng'   => (float)$row['lng'],
    'sats'  => (int)$row['sats'],
  ],
  'sim' => [
    'moduleOk' => true,
    'simReady' => true,
    'csq'      => 0,
    'cgatt'    => 1,
    'operator' => '',
  ],
  'events' => [],
];

echo json_encode(['ok' => true, 'data' => $data]);
