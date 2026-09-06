// joystick3d.js — 双 3D 摇杆（遥控形式）。
//
// 每个摇杆是独立自包含的 Three.js 场景：圆形底座 + 立柱 + 顶部摇杆球。拖拽球体在圆形
// 边界内移动，归一化为 (x,y)∈[-1,1] 经 ArmWS.sendJoy(side, x, y) 下发；松手回中即停。
//   - 左摇杆 side="L"：X=底座(9)，Y=左舵(8)
//   - 右摇杆 side="R"：X=夹取(6)，Y=右舵(7)
// 服务器把左右坐标合并为一条 JOY 四轴帧下发，串口层再按命令-应答门控串行化。
(function () {
  if (!window.THREE) {
    console.error("Three.js 未加载（检查网络/CDN）");
    return;
  }
  var THREE = window.THREE;

  // 单摇杆工厂：在自己的 scene/camera/renderer 中渲染，互不干扰。
  function makeJoystick(containerId, side) {
    var container = document.getElementById(containerId);
    if (!container) return;

    var scene = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
    camera.position.set(0, 4.2, 6.2);
    camera.lookAt(0, 1.0, 0);

    var renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setClearColor(0x000000, 0);
    container.appendChild(renderer.domElement);

    scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    var dir = new THREE.DirectionalLight(0xffffff, 0.9);
    dir.position.set(3, 8, 5);
    scene.add(dir);
    var dir2 = new THREE.DirectionalLight(0x6688ff, 0.35);
    dir2.position.set(-4, 3, -4);
    scene.add(dir2);

    var grid = new THREE.GridHelper(10, 20, 0x2a3340, 0x1a2230);
    grid.position.y = 0.001;
    scene.add(grid);

    // ---- 摇杆几何 ----
    var baseR = 1.15;
    var maxTilt = 0.55;
    var stickLen = 1.5;

    var joy = new THREE.Group();
    scene.add(joy);

    var baseMat = new THREE.MeshStandardMaterial({ color: 0x222a36, metalness: 0.3, roughness: 0.7 });
    var base = new THREE.Mesh(new THREE.CylinderGeometry(baseR, baseR * 1.1, 0.35, 48), baseMat);
    base.position.y = 0.175;
    joy.add(base);

    var ring = new THREE.Mesh(
      new THREE.TorusGeometry(baseR * 0.92, 0.04, 12, 48),
      new THREE.MeshStandardMaterial({ color: 0x4c8dff, emissive: 0x16335f, roughness: 0.5 })
    );
    ring.rotation.x = Math.PI / 2;
    ring.position.y = 0.36;
    joy.add(ring);

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

    function clamp(v, a, b) { return v < a ? a : v > b ? b : v; }

    function setNDC(e) {
      var r = renderer.domElement.getBoundingClientRect();
      pointer.x = ((e.clientX - r.left) / r.width) * 2 - 1;
      pointer.y = -((e.clientY - r.top) / r.height) * 2 + 1;
    }

    function planePoint() {
      raycaster.setFromCamera(pointer, camera);
      if (raycaster.ray.intersectPlane(dragPlane, hit)) {
        var dx = hit.x;          // 相对底座中心(0,0)
        var dz = hit.z;
        var x = dx / baseR;      // 右为正
        var y = -dz / baseR;     // 前(远离相机, -z)为正
        return { x: clamp(x, -1, 1), y: clamp(y, -1, 1) };
      }
      return null;
    }

    function applyJoy(x, y) {
      cur.x = x; cur.y = y;
      stick.rotation.z = -x * maxTilt;
      stick.rotation.x = y * maxTilt;
      var vx = container.querySelector(".jx");
      var vy = container.querySelector(".jy");
      if (vx) vx.textContent = x.toFixed(2);
      if (vy) vy.textContent = y.toFixed(2);

      var now = performance.now();
      // 前端节流 16ms（~60Hz），后端命令-应答门控 + 最新值合并会进一步抑制突发；
      // 真正瓶颈在串口字节传输，已通过 115200 波特率解决（见 README 延迟评估）。
      if (dragging && (Math.abs(x - lastSent.x) > 0.01 || Math.abs(y - lastSent.y) > 0.01 || now - lastSendT > 16)) {
        ArmWS.sendJoy(side, x, y);
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
      animateTo(0, 0);
      ArmWS.sendJoy(side, 0, 0); // 回中即停
      lastSent.x = 0; lastSent.y = 0;
    }

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
      var vx = container.querySelector(".jx");
      var vy = container.querySelector(".jy");
      if (vx) vx.textContent = cur.x.toFixed(2);
      if (vy) vy.textContent = cur.y.toFixed(2);
    }

    var el = renderer.domElement;
    el.addEventListener("pointerdown", onDown);
    window.addEventListener("pointermove", onMove);
    window.addEventListener("pointerup", onUp);

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
      renderer.render(scene, camera);
    }
    loop();
  }

  makeJoystick("joystick-left", "L");
  makeJoystick("joystick-right", "R");
})();
