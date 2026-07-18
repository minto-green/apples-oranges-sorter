#pragma once
#include <Arduino.h>

const char ik_dashboard_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Cycloidrive IK Solver Dashboard</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    <style>
        :root {
            --bg: #121212; --panel: #1e1e1e; --text: #e0e0e0; --accent: #00ff88; --border: #333;
        }
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: var(--bg); color: var(--text); margin: 0; padding: 20px; display: flex; flex-direction: column; gap: 20px; }
        h2, h3 { margin-top: 0; color: var(--accent); }
        .panel { background: var(--panel); padding: 20px; border-radius: 10px; border: 1px solid var(--border); box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
        
        /* Layout */
        .layout { display: grid; grid-template-columns: 350px 1fr; gap: 20px; max-width: 1200px; margin: 0 auto; width: 100%; }
        
        /* Controls */
        button { background: var(--accent); color: #000; border: none; padding: 12px 20px; border-radius: 5px; cursor: pointer; font-weight: bold; font-size: 1.1em; transition: 0.2s; width: 100%; }
        button:hover { background: #00cc6a; transform: translateY(-2px); }
        .btn-toggle { background: #444; color: #fff; margin-bottom: 15px; }
        .btn-toggle.active { background: #ff4444; color: #fff; }
        
        /* Sliders */
        .slider-group { background: #111; padding: 15px; border-radius: 8px; border: 1px solid var(--border); margin-bottom: 15px; }
        .slider-container { display: flex; align-items: center; margin-bottom: 10px; }
        .slider-container:last-child { margin-bottom: 0; }
        .slider-label { width: 70px; font-weight: bold; font-size: 0.9em; color: #aaa; }
        input[type=range] { flex-grow: 1; margin: 0 10px; accent-color: var(--accent); }
        .slider-val { width: 50px; text-align: right; font-family: monospace; color: var(--accent); font-weight: bold; }
        
        /* Canvas */
        canvas { display: block; background: #000; border: 1px solid var(--border); border-radius: 5px; width: 100%; height: 500px; }
        
        /* Outputs */
        .output-grid { display: grid; grid-template-columns: repeat(6, 1fr); gap: 10px; margin-top: 15px; margin-bottom: 20px;}
        .out-box { background: #000; border: 1px solid var(--border); padding: 10px; text-align: center; border-radius: 5px; }
        .out-lbl { font-size: 0.8em; color: #888; margin-bottom: 5px; }
        .out-val { font-family: monospace; font-size: 1.2em; color: #fff; }

        .warning { color: #ff4444; font-weight: bold; text-align: center; height: 20px; margin-bottom: 10px; }
    </style>
</head>
<body>

    <div class="layout">
        <!-- LEFT PANEL: Controls -->
        <div class="panel">
            <h2>Inverse Kinematics</h2>
            <p style="font-size: 0.9em; color: #aaa;">Target a 3D coordinate and the browser will compute the physical joint angles automatically.</p>
            
            <div class="slider-group">
                <h4 style="margin: 0 0 10px 0; color: #fff;">Target Position</h4>
                <div class="slider-container">
                    <div class="slider-label">X (Right)</div>
                    <input type="range" id="ik-x" min="-400" max="400" value="0" oninput="runIK()">
                    <div class="slider-val" id="val-x">0</div>
                </div>
                <div class="slider-container">
                    <div class="slider-label">Y (Up)</div>
                    <input type="range" id="ik-y" min="0" max="600" value="300" oninput="runIK()">
                    <div class="slider-val" id="val-y">300</div>
                </div>
                <div class="slider-container">
                    <div class="slider-label">Z (Fwd)</div>
                    <input type="range" id="ik-z" min="-400" max="400" value="200" oninput="runIK()">
                    <div class="slider-val" id="val-z">200</div>
                </div>
            </div>

            <div class="slider-group">
                <h4 style="margin: 0 0 10px 0; color: #fff;">Toolhead Orientation</h4>
                <div class="slider-container">
                    <div class="slider-label">Pitch</div>
                    <input type="range" id="ik-pitch" min="-90" max="270" value="90" oninput="runIK()" title="Angle of the toolhead relative to vertical. 90 = Flat forward.">
                    <div class="slider-val" id="val-pitch">90&deg;</div>
                </div>
                <div class="slider-container">
                    <div class="slider-label">Roll (J1)</div>
                    <input type="range" id="ik-roll" min="-360" max="360" value="0" oninput="runIK()">
                    <div class="slider-val" id="val-roll">0&deg;</div>
                </div>
            </div>

            <div class="slider-group">
                <h4 style="margin: 0 0 10px 0; color: #fff;">Peripherals</h4>
                <div class="slider-container">
                    <div class="slider-label">Extra (J6)</div>
                    <input type="range" id="ik-j6" min="-100" max="100" value="0" oninput="runIK()">
                    <div class="slider-val" id="val-j6">0&deg;</div>
                </div>
                <button id="btn-gripper" class="btn-toggle" onclick="toggleGripper()">Gripper: OPEN</button>
            </div>
            
            <div id="reach-warning" class="warning"></div>
        </div>

        <!-- RIGHT PANEL: Visualization & Execution -->
        <div class="panel">
            <canvas id="ikCanvas"></canvas>
            
            <h3 style="margin-top: 20px;">Calculated Joint Angles</h3>
            <div class="output-grid">
                <div class="out-box"><div class="out-lbl">J1 (Roll)</div><div class="out-val" id="out-j1">0.0</div></div>
                <div class="out-box"><div class="out-lbl">J2 (Pitch)</div><div class="out-val" id="out-j2">0.0</div></div>
                <div class="out-box"><div class="out-lbl">J3 (Elbow)</div><div class="out-val" id="out-j3">0.0</div></div>
                <div class="out-box"><div class="out-lbl">J4 (Shldr)</div><div class="out-val" id="out-j4">0.0</div></div>
                <div class="out-box"><div class="out-lbl">J5 (Base)</div><div class="out-val" id="out-j5">0.0</div></div>
                <div class="out-box"><div class="out-lbl">J6 (Extra)</div><div class="out-val" id="out-j6">0.0</div></div>
            </div>

            <button onclick="executeIKMove()" id="btn-exec">Execute IK Move</button>
            <div id="statusText" style="text-align: center; margin-top: 10px; color: #aaa; font-family: monospace;">Ready.</div>
        </div>
    </div>

    <script>
        // --- State Variables ---
        let solvedJoints = { j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0 };
        let isGripperClosed = false;

        function toggleGripper() {
            isGripperClosed = !isGripperClosed;
            const btn = document.getElementById('btn-gripper');
            if(isGripperClosed) {
                btn.innerText = "Gripper: CLOSED";
                btn.classList.add("active");
            } else {
                btn.innerText = "Gripper: OPEN";
                btn.classList.remove("active");
            }
            drawArm(); // Update visuals
        }

        // --- Kinematics Math ---
        function solveIK(tx, ty, tz, pitchDeg, rollDeg) {
            // Mechanical Limits
            const L1 = 133.3; // Base to Shoulder
            const L2 = 146.3; // Shoulder to Elbow
            const L3 = 146.3; // Elbow to Wrist
            const L4 = 185.4; // Wrist to Tool Tip

            // 1. Solve Base Rotation (J5)
            // atan2(x, z) gives rotation around Y axis.
            let J5_rad = Math.atan2(tx, tz);
            let J5 = J5_rad * 180 / Math.PI;

            // 2. Project target into the 2D plane created by the base rotation
            let r = Math.sqrt(tx*tx + tz*tz); // Forward distance from base center
            let y_local = ty - L1;            // Height relative to shoulder joint

            // 3. Find Wrist position based on requested Toolhead Pitch
            let pitch_rad = pitchDeg * Math.PI / 180;
            // Note: 0 deg = straight UP. 90 deg = straight FORWARD (+r).
            // So Sin maps to R, Cos maps to Y.
            let rw = r - L4 * Math.sin(pitch_rad);
            let yw = y_local - L4 * Math.cos(pitch_rad);

            // 4. Solve 2D 2-Link IK for Elbow (J3) and Shoulder (J4)
            let reachable = true;
            let D = (rw*rw + yw*yw - L2*L2 - L3*L3) / (2 * L2 * L3);
            if (D > 1 || D < -1) {
                reachable = false;
                D = Math.max(-1, Math.min(1, D)); // Clamp to max reach to still point at it
            }

            // Two solutions: Elbow Up or Elbow Down. We'll default to positive root.
            let J3_rad = Math.acos(D); 
            
            // Standard atan2 solves for angle relative to X-axis, but our 0-degree is UP (Y-axis).
            let theta = Math.atan2(rw, yw); // Angle of wrist from origin. Opp=R, Adj=Y.
            let phi = Math.atan2(L3 * Math.sin(J3_rad), L2 + L3 * Math.cos(J3_rad));

            let J4_rad = theta - phi;
            let J2_rad = pitch_rad - J4_rad - J3_rad;

            // 5. Convert to Degrees
            let J4 = J4_rad * 180 / Math.PI;
            let J3 = J3_rad * 180 / Math.PI;
            let J2 = J2_rad * 180 / Math.PI;

            // Clamp helper
            const clamp = (val, min, max) => Math.max(min, Math.min(max, val));

            return {
                j1: clamp(rollDeg, -360, 360),
                j2: clamp(J2, -100, 100),
                j3: clamp(J3, -100, 100),
                j4: clamp(J4, -100, 100),
                j5: clamp(J5, -360, 360),
                reachable: reachable
            };
        }

        // --- UI & Rendering Logic ---
        function runIK() {
            // Read inputs
            let tx = parseFloat(document.getElementById('ik-x').value);
            let ty = parseFloat(document.getElementById('ik-y').value);
            let tz = parseFloat(document.getElementById('ik-z').value);
            let pitch = parseFloat(document.getElementById('ik-pitch').value);
            let roll = parseFloat(document.getElementById('ik-roll').value);
            let j6 = parseFloat(document.getElementById('ik-j6').value);

            // Update UI Labels
            document.getElementById('val-x').innerText = tx;
            document.getElementById('val-y').innerText = ty;
            document.getElementById('val-z').innerText = tz;
            document.getElementById('val-pitch').innerText = pitch + "\u00B0";
            document.getElementById('val-roll').innerText = roll + "\u00B0";
            document.getElementById('val-j6').innerText = j6 + "\u00B0";

            // Move Target Sphere
            targetSphere.position.set(tx, ty, tz);

            // Run Solver
            let res = solveIK(tx, ty, tz, pitch, roll);
            solvedJoints = res;
            solvedJoints.j6 = j6;

            // Output values to UI
            document.getElementById('out-j1').innerText = res.j1.toFixed(1);
            document.getElementById('out-j2').innerText = res.j2.toFixed(1);
            document.getElementById('out-j3').innerText = res.j3.toFixed(1);
            document.getElementById('out-j4').innerText = res.j4.toFixed(1);
            document.getElementById('out-j5').innerText = res.j5.toFixed(1);
            document.getElementById('out-j6').innerText = j6.toFixed(1);

            if(!res.reachable) {
                document.getElementById('reach-warning').innerText = "TARGET OUT OF REACH! Limiting to max extension.";
                targetSphere.material.color.setHex(0xff0000);
            } else {
                document.getElementById('reach-warning').innerText = "";
                targetSphere.material.color.setHex(0x00ff88);
            }

            drawArm(); // Refresh Three.js Model
        }

        function executeIKMove() {
            let cmd = `${solvedJoints.j1.toFixed(1)},${solvedJoints.j2.toFixed(1)},${solvedJoints.j3.toFixed(1)},${solvedJoints.j4.toFixed(1)},${solvedJoints.j5.toFixed(1)},${solvedJoints.j6.toFixed(1)},${isGripperClosed ? 'C' : 'O'}`;
            
            document.getElementById('statusText').innerText = "Sending: " + cmd;
            
            fetch('/api/sequence', {
                method: 'POST',
                headers: { 'Content-Type': 'text/plain' },
                body: cmd
            }).then(response => {
                if(response.ok) document.getElementById('statusText').innerText = "Move Executed: " + cmd;
                else document.getElementById('statusText').innerText = "Network Error!";
            }).catch(err => document.getElementById('statusText').innerText = "Connection Failed.");
        }


        // --- 3D WebGL Render (Three.js) ---
        const canvas = document.getElementById('ikCanvas');
        const renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
        
        // Dynamically size based on container
        renderer.setSize(canvas.clientWidth, canvas.clientHeight);
        window.addEventListener('resize', () => {
            renderer.setSize(canvas.clientWidth, canvas.clientHeight);
            camera.aspect = canvas.clientWidth / canvas.clientHeight;
            camera.updateProjectionMatrix();
        });

        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x111111);
        
        const camera = new THREE.PerspectiveCamera(45, canvas.clientWidth / canvas.clientHeight, 1, 2000);
        camera.position.set(500, 400, 600); 
        
        const controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.target.set(0, 250, 0);
        controls.update();
        
        // Lighting & Grid
        scene.add(new THREE.AmbientLight(0x404040, 1.5));
        const dirLight = new THREE.DirectionalLight(0xffffff, 1);
        dirLight.position.set(200, 500, 300);
        scene.add(dirLight);
        scene.add(new THREE.GridHelper(1000, 20, 0x00ff88, 0x333333));
        
        // Target Sphere
        const targetGeo = new THREE.SphereGeometry(15, 16, 16);
        const targetMat = new THREE.MeshBasicMaterial({color: 0x00ff88, wireframe: true});
        const targetSphere = new THREE.Mesh(targetGeo, targetMat);
        scene.add(targetSphere);

        // Robot Arm Construction
        const L1 = 133.3, L2 = 146.3, L3 = 146.3, L4 = 185.4;
        
        function createLink(length, color) {
            const group = new THREE.Group();
            const geo = new THREE.CylinderGeometry(12, 12, length, 16);
            geo.translate(0, length / 2, 0);
            const mat = new THREE.MeshPhongMaterial({ color: color });
            group.add(new THREE.Mesh(geo, mat));
            
            const jointGeo = new THREE.SphereGeometry(16, 16, 16);
            const jointMat = new THREE.MeshPhongMaterial({ color: 0x444444 });
            group.add(new THREE.Mesh(jointGeo, jointMat));
            return group;
        }
        
        const j5Group = new THREE.Group(); scene.add(j5Group);
        const j5Link = createLink(L1, 0x555555); j5Group.add(j5Link);
        
        const j4Group = new THREE.Group(); j4Group.position.y = L1; j5Group.add(j4Group);
        const j4Link = createLink(L2, 0x888888); j4Group.add(j4Link);
        
        const j3Group = new THREE.Group(); j3Group.position.y = L2; j4Group.add(j3Group);
        const j3Link = createLink(L3, 0xaaaaaa); j3Group.add(j3Link);
        
        const j2Group = new THREE.Group(); j2Group.position.y = L3; j3Group.add(j2Group);
        const j2Link = createLink(L4 - 41.2, 0x00ff88); j2Group.add(j2Link);
        
        const j1Group = new THREE.Group(); j1Group.position.y = L4 - 41.2; j2Group.add(j1Group);
        
        const gripperLeft = new THREE.Mesh(new THREE.BoxGeometry(4, 41.2, 10), new THREE.MeshPhongMaterial({color: 0xcccccc}));
        gripperLeft.position.set(-15, 20.6, 0);
        const gripperRight = new THREE.Mesh(new THREE.BoxGeometry(4, 41.2, 10), new THREE.MeshPhongMaterial({color: 0xcccccc}));
        gripperRight.position.set(15, 20.6, 0);
        const gripperBase = new THREE.Mesh(new THREE.BoxGeometry(40, 10, 20), new THREE.MeshPhongMaterial({color: 0x888888}));
        j1Group.add(gripperBase, gripperLeft, gripperRight);

        function drawArm() {
            j5Group.rotation.y = -(solvedJoints.j5) * (Math.PI / 180);
            j4Group.rotation.x =  (solvedJoints.j4) * (Math.PI / 180);
            j3Group.rotation.x =  (solvedJoints.j3) * (Math.PI / 180);
            j2Group.rotation.x =  (solvedJoints.j2) * (Math.PI / 180);
            j1Group.rotation.y = (solvedJoints.j1 - 90) * (Math.PI / 180);
            
            gripperLeft.position.x = isGripperClosed ? -5 : -15;
            gripperRight.position.x = isGripperClosed ? 5 : 15;
        }
        
        function animate() {
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        }

        // Init
        runIK();
        animate(); 
    </script>
</body>
</html>
)rawliteral";