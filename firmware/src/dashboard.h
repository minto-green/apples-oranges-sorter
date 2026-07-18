#pragma once
#include <Arduino.h>

const char dashboard_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Cycloidrive 6-DOF Arm Dashboard</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    <style>
        :root {
            --bg: #121212; --panel: #1e1e1e; --text: #e0e0e0; --accent: #00ff88; --border: #333;
        }
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: var(--bg); color: var(--text); margin: 0 auto; padding: 20px; display: flex; flex-direction: column; gap: 20px; max-width: 900px; }
        h2, h3 { margin-top: 0; color: var(--accent); }
        .panel { background: var(--panel); padding: 20px; border-radius: 10px; border: 1px solid var(--border); box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
        
        /* Grid Layouts */
        .full-width { grid-column: 1 / -1; }
        
        /* Controls */
        textarea { width: 100%; height: 120px; background: #000; color: #00ff88; font-family: monospace; border: 1px solid var(--border); padding: 10px; border-radius: 5px; resize: vertical; box-sizing: border-box; }
        button { background: var(--accent); color: #000; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; font-weight: bold; margin-top: 10px; transition: 0.2s; }
        button:hover { background: #00cc6a; transform: translateY(-2px); }
        
        /* Sliders */
        .slider-container { display: flex; align-items: center; margin-bottom: 15px; }
        .slider-label { width: 50px; font-weight: bold; }
        input[type=range] { flex-grow: 1; margin: 0 15px; accent-color: var(--accent); }
        .slider-val { width: 50px; text-align: right; font-family: monospace; }
        
        /* Sequence Output */
        .sequence-box { background: #000; border: 1px solid var(--border); padding: 15px; text-align: center; font-size: 1.2em; font-family: monospace; letter-spacing: 2px; border-radius: 5px; margin-top: 15px; user-select: all; cursor: pointer; color: #00ff88; white-space: pre-wrap; line-height: 1.5; }
        .sequence-box:hover { background: #111; }

        /* Canvas Live View */
        canvas { display: block; margin: 0 auto 20px auto; background: #000; border: 1px solid var(--border); border-radius: 5px; width: 100%; max-width: 800px; height: auto; aspect-ratio: 8 / 5; }
        
        /* Status Bar */
        .status-bar { background: #000; color: #aaa; padding: 10px; border-radius: 5px; font-family: monospace; border: 1px solid var(--border); }
        .status-active { color: var(--accent); font-weight: bold; }
    </style>
</head>
<body>

    <div class="panel full-width">
        <h2>Cycloidrive System Control</h2>
        <div class="status-bar" id="statusText">System Idle. Waiting for commands...</div>
    </div>

    <div class="panel">
        <h3>Live Pose Configuration</h3>
        <canvas id="armCanvas"></canvas>
        <p style="text-align:center; font-size: 0.8em; color: #888;">3D WebGL View (Drag to rotate, Scroll to zoom)</p>

        <h3 style="margin-top: 20px;">Manual Graphic Control</h3>
        <div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; margin-bottom: 20px; background: #111; padding: 15px; border-radius: 8px; border: 1px solid var(--border);">
            <div style="text-align: center;">
                <div style="color: #888; margin-bottom: 10px;">[ J4: Shoulder ]</div>
                <button onclick="nudge(3, -5)" style="margin-top:0;">-5&deg;</button>
                <button onclick="nudge(3, 5)" style="margin-top:0;">+5&deg;</button>
            </div>
            <div style="text-align: center;">
                <div style="color: #888; margin-bottom: 10px;">[ J3: Elbow ]</div>
                <button onclick="nudge(2, -5)" style="margin-top:0;">-5&deg;</button>
                <button onclick="nudge(2, 5)" style="margin-top:0;">+5&deg;</button>
            </div>
            <div style="text-align: center;">
                <div style="color: #888; margin-bottom: 10px;">[ Gripper ]</div>
                <button onclick="setGripper('C')" style="margin-top:0;">Close</button>
                <button onclick="setGripper('O')" style="margin-top:0;">Open</button>
            </div>
        </div>

        <div id="sliders" style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px;"></div> 
        
        <h4 style="margin-top: 25px;">Motion Parameters</h4>
        <div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px;">
            <div class="slider-container">
                <div class="slider-label" style="width: 35px;">SPD</div>
                <input type="range" id="speedParam" min="1" max="50" value="50" oninput="updateUI()">
                <div class="slider-val" id="speedParam-val" style="width: 35px;">50</div>
            </div>
            <div class="slider-container">
                <div class="slider-label" style="width: 35px;">ACC</div>
                <input type="range" id="accelParam" min="1" max="25" value="25" oninput="updateUI()">
                <div class="slider-val" id="accelParam-val" style="width: 35px;">25</div>
            </div>
            <div class="slider-container">
                <div class="slider-label" style="width: 35px;">DLY</div>
                <input type="range" id="delayParam" min="0" max="5000" step="50" value="0" oninput="updateUI()">
                <div class="slider-val" id="delayParam-val" style="width: 45px;">0ms</div>
            </div>
        </div>

        <h4 style="margin-top: 15px;">Copy-Paste Sequence Generator</h4>
        <div class="sequence-box" id="sequenceOutput" onclick="copySequence()" title="Click to copy">0,0,0,0,0,0,X</div>
                <div style="display: flex; gap: 10px; margin-top: 10px;">
            <button onclick="sendLivePose()" style="flex: 1;">Execute This Pose</button>
            <button onclick="homeArm()" style="flex: 1; background: #ff4444; color: #fff;">Home Arm (0,0,0...)</button>
            <button onclick="postCommand('STOP')" style="flex: 1; background: #cc0000; color: #fff;">🛑 CANCEL</button>
            <button onclick="runReadySequence()" style="flex: 1; background: #ffaa00; color: #fff;">Run Ready Sequence</button>
        </div>
    </div>

    <div class="panel">
        <h3>Sequence Execution</h3>
        <p>Paste your multi-line commands here (e.g. <code>90,45,-45,0,0,0,C</code> or <code>D:500</code>).</p>
        <textarea id="sequenceInput" placeholder="-90,-90,-45,-45,0,0,X&#10;D:500&#10;90,-95,45,90,0,0,C"></textarea>
        <div style="display: flex; gap: 10px; margin-top: 10px;">
            <button onclick="executeSequence()" style="flex: 2;">Execute Sequence</button>
            <button onclick="document.getElementById('sequenceInput').value=''" style="flex: 1; background:#444; color:#fff;">Clear Box</button>
        </div>
    </div>

    <script>
        // --- Slider & UI Logic ---
        const joints = [
            { id: 'j1', label: 'J1 (Roll)', min: -360, max: 360, val: 0 },
            { id: 'j2', label: 'J2 (Pitch)', min: -100, max: 100, val: 0 },
            { id: 'j3', label: 'J3 (Elbow)', min: -100, max: 100, val: 0 },
            { id: 'j4', label: 'J4 (Shldr)', min: -100, max: 100, val: 0 },
            { id: 'j5', label: 'J5 (Base)', min: -360, max: 360, val: 0 },
            { id: 'j6', label: 'J6 (Extra)', min: -100, max: 100, val: 0 }
        ];
        
        let isGripperClosed = false;

        const sliderDiv = document.getElementById('sliders');
        joints.forEach((j, index) => {
            sliderDiv.innerHTML += `
                <div class="slider-container">
                    <div class="slider-label">${j.id.toUpperCase()}</div>
                    <input type="range" id="${j.id}" min="${j.min}" max="${j.max}" value="${j.val}" oninput="updateUI()">
                    <div class="slider-val" id="${j.id}-val">${j.val}&deg;</div>
                </div>
            `;
        });

        function updateUI() {
            let seq = [];
            joints.forEach(j => {
                let val = document.getElementById(j.id).value;
                document.getElementById(j.id + '-val').innerText = val + '\u00B0';
                seq.push(val);
            });
            
            let speed = document.getElementById('speedParam').value;
            let accel = document.getElementById('accelParam').value;
            let delay = document.getElementById('delayParam').value;
            
            document.getElementById('speedParam-val').innerText = speed;
            document.getElementById('accelParam-val').innerText = accel;
            document.getElementById('delayParam-val').innerText = delay + 'ms';
            
            let gripperChar = isGripperClosed ? 'C' : 'O';
            let moveString = seq.join(',') + ',' + gripperChar;
            
            let lines = [];
            if (speed != 50) lines.push("S:" + speed);
            if (accel != 25) lines.push("A:" + accel);
            lines.push(moveString);
            if (delay > 0) lines.push("D:" + delay);
            
            document.getElementById('sequenceOutput').innerText = lines.join('\n');
            drawArm();
        }

        function copySequence() {
            navigator.clipboard.writeText(document.getElementById('sequenceOutput').innerText);
            alert("Sequence copied to clipboard!");
        }

        function nudge(jointIndex, amount) {
            let j = joints[jointIndex];
            let el = document.getElementById(j.id);
            let newVal = parseInt(el.value) + amount;
            if (newVal > j.max) newVal = j.max;
            if (newVal < j.min) newVal = j.min;
            el.value = newVal;
            updateUI();
        }

        function homeArm() {
            joints.forEach(j => {
                document.getElementById(j.id).value = 0;
            });
            document.getElementById('speedParam').value = 50;
            document.getElementById('accelParam').value = 25;
            document.getElementById('delayParam').value = 0;
            updateUI();
            postCommand('S:50\nA:25\n0,0,0,0,0,0,X');
        }

        function setGripper(state) {
            isGripperClosed = (state === 'C');
            postCommand(state === 'C' ? 'TC' : 'TO');
            updateUI(); // Refresh sequence generator and canvas
        }

        function runReadySequence() {
            let seq = "65,52,37,49,52,0,C\nD: 100\n0,0,0,0,0,0,O\nD: 100\n65,52,37,49,-52,0,C\nD: 100\n0,0,0,0,0,0,O";
            postCommand(seq);
        }

        // --- 3D WebGL Render (Three.js Forward Kinematics) ---
        const canvas = document.getElementById('armCanvas');
        const renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
        renderer.setSize(800, 500);
        
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x111111);
        
        const camera = new THREE.PerspectiveCamera(45, 800 / 500, 1, 2000);
        camera.position.set(400, 400, 800); // Pulled back slightly for a better framing
        
        const controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.target.set(0, 300, 0);
        controls.update();
        
        // Lighting & Grid
        scene.add(new THREE.AmbientLight(0x404040, 1.5));
        const dirLight = new THREE.DirectionalLight(0xffffff, 1);
        dirLight.position.set(200, 500, 300);
        scene.add(dirLight);
        scene.add(new THREE.GridHelper(800, 20, 0x00ff88, 0x444444));
        
        // Robot Arm Construction
        const L1 = 133.3, L2 = 146.3, L3 = 146.4, L4 = 185.4;
        
        function createLink(length, color) {
            const group = new THREE.Group();
            const geo = new THREE.CylinderGeometry(12, 12, length, 16);
            geo.translate(0, length / 2, 0); // Shift so origin is at the base joint
            const mat = new THREE.MeshPhongMaterial({ color: color });
            group.add(new THREE.Mesh(geo, mat));
            
            const jointGeo = new THREE.SphereGeometry(16, 16, 16);
            const jointMat = new THREE.MeshPhongMaterial({ color: 0xff4444 });
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
        const j2Link = createLink(L4 - 30, 0x00ff88); j2Group.add(j2Link);
        
        const j1Group = new THREE.Group(); j1Group.position.y = L4 - 30; j2Group.add(j1Group);
        
        const gripperLeft = new THREE.Mesh(new THREE.BoxGeometry(6, 30, 20), new THREE.MeshPhongMaterial({color: 0xcccccc}));
        gripperLeft.position.set(-15, 15, 0);
        const gripperRight = new THREE.Mesh(new THREE.BoxGeometry(6, 30, 20), new THREE.MeshPhongMaterial({color: 0xcccccc}));
        gripperRight.position.set(15, 15, 0);
        const gripperBase = new THREE.Mesh(new THREE.BoxGeometry(40, 10, 20), new THREE.MeshPhongMaterial({color: 0x888888}));
        j1Group.add(gripperBase, gripperLeft, gripperRight);

        function drawArm() {
            j5Group.rotation.y = -(parseInt(document.getElementById('j5').value)) * (Math.PI / 180);
            j4Group.rotation.x =  (parseInt(document.getElementById('j4').value)) * (Math.PI / 180);
            j3Group.rotation.x =  (parseInt(document.getElementById('j3').value)) * (Math.PI / 180);
            j2Group.rotation.x =  (parseInt(document.getElementById('j2').value)) * (Math.PI / 180);
            j1Group.rotation.y = (parseInt(document.getElementById('j1').value) - 90) * (Math.PI / 180);
            
            gripperLeft.position.x = isGripperClosed ? -5 : -15;
            gripperRight.position.x = isGripperClosed ? 5 : 15;
        }
        
        function animate() {
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        }
        animate(); // Start loop

        // --- ESP32 Communication ---
        function executeSequence() {
            let seq = document.getElementById('sequenceInput').value;
            postCommand(seq);
        }

        function sendLivePose() {
            let seq = document.getElementById('sequenceOutput').innerText;
            postCommand(seq);
        }

                function postCommand(commandString) {
            document.getElementById('statusText').innerHTML = "<span class='status-active'>Sending sequence to ESP32...</span>";
            fetch('/api/sequence', {
                method: 'POST',
                headers: { 'Content-Type': 'text/plain' },
                body: commandString
            }).then(response => {
                if(response.ok) {
                    if (commandString === 'STOP') {
                        document.getElementById('statusText').innerHTML = "<span style='color: #ff0000; font-weight: bold;'>CANCELLED AND HALTED.</span>";
                    } else {
                        document.getElementById('statusText').innerText = "Sequence received and executing.";
                    }
                }
                else document.getElementById('statusText').innerText = "Error sending command.";
            }).catch(err => document.getElementById('statusText').innerText = "Network Error!");
        }

                // Poll status every 400 milliseconds
        setInterval(() => {
            fetch('/api/status').then(r => r.json()).then(data => {
                if(data && data.angles) {
                    let anglesStr = "[ " + data.angles.map(a => a.toFixed(1) + "&deg;").join(", ") + " ]";
                    if (data.status === "HALTED") {
                        document.getElementById('statusText').innerHTML = "<span style='color: #ff0000; font-weight: bold;'>HALTED:</span> " + anglesStr;
                    } else if(data.status === "MOVING") {
                        document.getElementById('statusText').innerHTML = "<span style='color: #00ff88; font-weight: bold;'>Moving:</span> " + anglesStr;
                    } else {
                        document.getElementById('statusText').innerHTML = "Idle. Current: " + anglesStr;
                    }
                }
            }).catch(e => {});
        }, 400);

        // Init
        updateUI();
    </script>
</body>
</html>
)rawliteral";