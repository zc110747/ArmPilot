// joystick3d.js — Three.js 3D 摇杆（网页端 3D 控制的核心）。
//
// 形状与硬件摇杆一致：圆形底座 + 立柱 + 顶部摇杆球。拖拽球体在圆形边界内移动，
// 归一化为 (x,y)∈[-1,1] 经 ArmWS.sendJoy 下发；松手回中(x=0,y=0)即停。
// 同时维护共享场景 window.ArmScene，供 arm3d.js 挂载实时姿态模型。
(function () {
  if (!window.THREE) {
    console.error("Three.js 未加载（检查网络/CDN）");
    return;
  }
  var THREE = window.THREE;

  var container = document.getElementById("joystick-container");
  var scene = new THREE.Scene();

  var camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
  camera.position.set(0, 4.2, 6.2);
  camera.lookAt(0, 1.0, 0);

  var renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setClearColor(0x000000, 0);
  container.appendChild(renderer.domElement);

  // 灯光
  scene.add(new THREE.AmbientLight(0xffffff, 0.55));
  var dir = new THREE.DirectionalLight(0xffffff, 0.9);
  dir.position.set(3, 8, 5);
  scene.add(dir);
  var dir2 = new THREE.DirectionalLight(0x6688ff, 0.35);
  dir2.position.set(-4, 3, -4);
  scene.add(dir2);

  // 地面网格
  var grid = new THREE.GridHelper(10, 20, 0x2a3340, 0x1a2230);
  grid.position.y = 0.001;
  scene.add(grid);

  // ---- 摇杆（位于场景左侧 x=-2.2）----
  var JX = -2.2;
  var baseR = 1.15;       // 底座半径
  var maxTilt = 0.55;     // 最大倾角(rad)
  var stickLen = 1.5;

  var joy = new THREE.Group();
  joy.position.set(JX, 0, 0);
  scene.add(joy);

  // 底座
  var baseMat = new THREE.MeshStandardMaterial({ color: 0x222a36, metalness: 0.3, roughness: 0.7 });
  var base = new THREE.Mesh(new THREE.CylinderGeometry(baseR, baseR * 1.1, 0.35, 48), baseMat);
  base.position.y = 0.175;
  joy.add(base);

  // 边界环（指示活动范围）
  var ring = new THREE.Mesh(
    new THREE.TorusGeometry(baseR * 0.92, 0.04, 12, 48),
    new THREE.MeshStandardMaterial({ color: 0x4c8dff, emissive: 0x16335f, roughness: 0.5 })
  );
  ring.rotation.x = Math.PI / 2;
  ring.position.y = 0.36;
  joy.add(ring);

  // 摇杆立柱 + 球（可绕底座中心俯仰/偏转）
  var stick = new THREE.Group();
  stick.position.y = 0.35;
  joy.add(stick);

  var shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(0.10, 0.13, stickLen, 20),
    new THREE.MeshStandardMaterial({ color: 0x3a4658, metalness: 0.5, roughness: 0.4 })
  );
  shaft.position.y = stickLen / 2;
  stick.add(shaft);

  var knob = new THREE.Mesh(
    new THREE.SphereGeometry(0.42, 32, 32),
    new THREE.MeshStandardMaterial({ color: 0x4c8dff, emissive: 0x10325f, metalness: 0.3, roughness: 0.35 })
  );
  knob.position.y = stickLen;
  stick.add(knob);

  // ---- 交互 ----
  var raycaster = new THREE.Raycaster();
  var pointer = new THREE.Vector2();
  var dragPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -(0.35 + stickLen * 0.7));
  var hit = new THREE.Vector3();
  var dragging = false;
  var cur = { x: 0, y: 0 };
  var lastSent = { x: 0, y: 0 };
  var lastSendT = 0;

  function setNDC(e) {
    var r = renderer.domElement.getBoundingClientRect();
    pointer.x = ((e.clientX - r.left) / r.width) * 2 - 1;
    pointer.y = -((e.clientY - r.top) / r.height) * 2 + 1;
  }

  function planePoint() {
    raycaster.setFromCamera(pointer, camera);
    if (raycaster.ray.intersectPlane(dragPlane, hit)) {
      // 相对底座中心(joy.position)的水平偏移
      var dx = hit.x - JX;
      var dz = hit.z - 0;
      var x = dx / baseR;          // 右为正
      var y = -dz / baseR;         // 前(远离相机, -z)为正
      return { x: clamp(x, -1, 1), y: clamp(y, -1, 1) };
    }
    return null;
  }

  function clamp(v, a, b) { return v < a ? a : v > b ? b : v; }

  function applyJoy(x, y) {
    cur.x = x; cur.y = y;
    // 旋转：x -> 绕Z偏转(左右)，y -> 绕X俯仰(前后)
    stick.rotation.z = -x * maxTilt;
    stick.rotation.x = y * maxTilt;
    // 更新读数
    var vx = document.getElementById("val-x");
    var vy = document.getElementById("val-y");
    if (vx) vx.textContent = x.toFixed(2);
    if (vy) vy.textContent = y.toFixed(2);

    var now = performance.now();
    if (dragging && (Math.abs(x - lastSent.x) > 0.01 || Math.abs(y - lastSent.y) > 0.01 || now - lastSendT > 60)) {
      ArmWS.sendJoy(x, y);
      lastSent.x = x; lastSent.y = y; lastSendT = now;
    }
  }

  function onDown(e) {
    setNDC(e);
    raycaster.setFromCamera(pointer, camera);
    var hits = raycaster.intersectObject(knob, false);
    if (hits.length > 0) {
      dragging = true;
      e.preventDefault();
    }
  }
  function onMove(e) {
    if (!dragging) return;
    setNDC(e);
    var p = planePoint();
    if (p) applyJoy(p.x, p.y);
  }
  function onUp() {
    if (!dragging) return;
    dragging = false;
    // 回中并停
    animateTo(0, 0);
    ArmWS.sendJoy(0, 0);
    lastSent.x = 0; lastSent.y = 0;
  }

  // 松手回中动画
  var animTarget = null;
  function animateTo(x, y) { animTarget = { x: x, y: y }; }
  function tickAnim() {
    if (!animTarget) return;
    cur.x += (animTarget.x - cur.x) * 0.25;
    cur.y += (animTarget.y - cur.y) * 0.25;
    if (Math.abs(cur.x - animTarget.x) < 0.01 && Math.abs(cur.y - animTarget.y) < 0.01) {
      cur.x = animTarget.x; cur.y = animTarget.y; animTarget = null;
    }
    stick.rotation.z = -cur.x * maxTilt;
    stick.rotation.x = cur.y * maxTilt;
    var vx = document.getElementById("val-x");
    var vy = document.getElementById("val-y");
    if (vx) vx.textContent = cur.x.toFixed(2);
    if (vy) vy.textContent = cur.y.toFixed(2);
  }

  var el = renderer.domElement;
  el.addEventListener("pointerdown", onDown);
  window.addEventListener("pointermove", onMove);
  window.addEventListener("pointerup", onUp);

  // ---- 共享场景（arm3d.js 使用）----
  window.ArmScene = {
    scene: scene,
    THREE: THREE,
    camera: camera,
    addTick: function (fn) { ticks.push(fn); },
  };
  var ticks = [];

  // ---- 渲染循环 ----
  function resize() {
    var w = container.clientWidth, h = container.clientHeight;
    if (w === 0 || h === 0) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  window.addEventListener("resize", resize);
  resize();

  function loop() {
    requestAnimationFrame(loop);
    tickAnim();
    for (var i = 0; i < ticks.length; i++) ticks[i]();
    renderer.render(scene, camera);
  }
  loop();
})();
