// arm3d.js — 实时姿态模型（摇杆模式下的 3D 反馈）。
// 订阅 ArmWS 的 serial 消息中的 angles 字段（由服务器从设备回显解析），
// 把 S6/S7/S8/S9 角度映射到 3D 机械臂各关节，直观显示当前姿态。
(function () {
  if (!window.ArmScene || !window.ArmWS) {
    console.warn("ArmScene / ArmWS 尚未就绪");
    return;
  }
  var THREE = window.ArmScene.THREE;
  var scene = window.ArmScene.scene;

  var AX = 2.4; // 模型放在场景右侧
  var mat = function (c) { return new THREE.MeshStandardMaterial({ color: c, metalness: 0.35, roughness: 0.55 }); };

  var root = new THREE.Group();
  root.position.set(AX, 0, 0);
  scene.add(root);

  // 底座（绕 Y 旋转 = S9）
  var base = new THREE.Mesh(new THREE.CylinderGeometry(0.7, 0.85, 0.3, 32), mat(0x2a3340));
  base.position.y = 0.15;
  root.add(base);
  var baseTop = new THREE.Mesh(new THREE.CylinderGeometry(0.45, 0.6, 0.25, 32), mat(0x39455a));
  baseTop.position.y = 0.4;
  root.add(baseTop);

  var yaw = new THREE.Group();      // 绕Y = S9 (30..150 -> -60..60deg)
  yaw.position.y = 0.5;
  root.add(yaw);

  var shoulder = new THREE.Group(); // 绕Z = S8
  shoulder.position.y = 0.1;
  yaw.add(shoulder);
  var upper = new THREE.Mesh(new THREE.BoxGeometry(0.22, 1.2, 0.22), mat(0x4c8dff));
  upper.position.y = 0.6;
  shoulder.add(upper);

  var elbow = new THREE.Group();    // 绕Z = S7
  elbow.position.y = 1.2;
  shoulder.add(elbow);
  var fore = new THREE.Mesh(new THREE.BoxGeometry(0.18, 1.0, 0.18), mat(0x6aa0ff));
  fore.position.y = 0.5;
  elbow.add(fore);

  var wrist = new THREE.Group();    // 夹取 = S6
  wrist.position.y = 1.0;
  elbow.add(wrist);
  var grip = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.2, 0.3), mat(0x39455a));
  wrist.add(grip);
  var clawL = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.3, 0.18), mat(0xd29922));
  clawL.position.set(-0.12, 0.2, 0);
  var clawR = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.3, 0.18), mat(0xd29922));
  clawR.position.set(0.12, 0.2, 0);
  wrist.add(clawL); wrist.add(clawR);

  function deg2rad(d) { return d * Math.PI / 180; }

  // 角度->姿态映射（仅展示趋势，非严格 DH 正解）
  function update(a) {
    if (!a || !a.ok) return;
    yaw.rotation.y = deg2rad((a.s9 - 90)) ;           // 底座 90 为中位
    shoulder.rotation.z = deg2rad((90 - a.s8)) * 0.6; // 左舵
    elbow.rotation.z = deg2rad((a.s7 - 120)) * 0.6;   // 右舵
    var open = (a.s6 - 40) / 85;                      // 夹取 40..125
    clawL.position.x = -0.12 - open * 0.10;
    clawR.position.x = 0.12 + open * 0.10;
  }

  window.ArmScene.addTick(function () {}); // 占位，保证渲染循环存在

  ArmWS.on("serial", function (data) {
    // data 可能是字符串(line) 或 {line, angles}
    if (data && data.angles) update(data.angles);
  });
})();
