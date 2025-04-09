document.addEventListener("DOMContentLoaded", (event) => {
    let ws;
    const connectWebSocket = () => {
        ws = new WebSocket("ws://" + location.hostname + ":81/");

        ws.onopen = () => {
            console.log("WebSocket connected");
            const connectionStatus =
                document.getElementById("connectionStatus");
            connectionStatus.className = "status-indicator status-success";
            connectionStatus.innerHTML =
                '<div class="status-dot"></div><span>Connected</span>';

            ws.send("keepAlive");
            setInterval(() => ws.send("keepAlive"), 30000);
        };

        ws.onmessage = function (event) {
            const data = JSON.parse(event.data);

            quatDataQ0.push(data.q0);
            quatDataQ1.push(data.q1);
            quatDataQ2.push(data.q2);
            quatDataQ3.push(data.q3);

            // Update accelerometer data
            accelDataX.push(data.ax);
            accelDataY.push(data.ay);
            accelDataZ.push(data.az);
            accelDataA.push(data.aa);

            // Update gyroscope data
            gyroDataX.push(data.gx);
            gyroDataY.push(data.gy);
            gyroDataZ.push(data.gz);

            rollData.push(data.roll);
            pitchData.push(data.pitch);
            yawData.push(data.yaw);

            const rx = data.rx,
                ry = data.ry,
                rz = data.rz;

            const rMag = Math.sqrt(rx * rx + ry * ry + rz * rz);
            const rAngle = ((Math.atan2(ry, rx) * 180) / Math.PI + 360) % 360;
            console.log("rMag:", rMag, "rAngle:", rAngle);

            rMagnitudeData.push(rMag);
            rAngleData.push(rAngle);

            // Update calibration loss data
            calibrationLossData.push(data.cl);

            // Common timestamp for all charts
            labels.push("");

            // Keep the data arrays at a fixed length
            if (accelDataX.length > 300) {
                quatDataQ0.shift();
                quatDataQ1.shift();
                quatDataQ2.shift();
                quatDataQ3.shift();

                accelDataX.shift();
                accelDataY.shift();
                accelDataZ.shift();
                accelDataA.shift();

                gyroDataX.shift();
                gyroDataY.shift();
                gyroDataZ.shift();

                rollData.shift();
                pitchData.shift();
                yawData.shift();

                calibrationLossData.shift();

                rMagnitudeData.shift();
                rAngleData.shift();

                labels.shift();
            }

            // Update all charts
            accelChart.update();
            gyroChart.update();
            calibrationChart.update();
            quatChart.update();
            eulerChart.update();
            rChart.data.labels = [rAngle.toFixed(1) + "°"];
            rChart.data.datasets[0].data = [rMag];
            rChart.update();
            // Update fall detection status
            const fallAlert = document.getElementById("fallAlert");
            if (data.fall) {
                fallAlert.className = "status-indicator status-danger";
                fallAlert.innerHTML =
                    '<div class="status-dot"></div><span>FALL DETECTED!</span>';
            } else {
                fallAlert.className = "status-indicator status-success";
                fallAlert.innerHTML =
                    '<div class="status-dot"></div><span>No Fall Detected</span>';
            }
        };

        ws.onclose = () => {
            console.log("WebSocket disconnected, retrying...");
            const connectionStatus =
                document.getElementById("connectionStatus");
            connectionStatus.className = "status-indicator status-warning";
            connectionStatus.innerHTML =
                '<div class="status-dot"></div><span>Reconnecting...</span>';

            setTimeout(connectWebSocket, 1000);
        };

        ws.onerror = (error) => {
            console.log("WebSocket Error: ", error);
            const connectionStatus =
                document.getElementById("connectionStatus");
            connectionStatus.className = "status-indicator status-danger";
            connectionStatus.innerHTML =
                '<div class="status-dot"></div><span>Connection Error</span>';

            ws.close();
        };
    };

    connectWebSocket();

    const betaSlider = document.getElementById("betaSlider");
    const betaValueDisplay = document.getElementById("betaValue");

    betaSlider.addEventListener("input", function () {
        const beta = parseFloat(betaSlider.value).toFixed(1);
        betaValueDisplay.textContent = beta;

        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send("setBeta:" + beta);
        }
    });

    document.getElementById("toggleR").addEventListener("change", function () {
        const enabled = this.checked;
        ws.send("toggleR:" + (enabled ? "on" : "off"));
    });

    document
        .getElementById("toggleQuat")
        .addEventListener("change", function () {
            const enabled = this.checked;
            ws.send("toggleQuat:" + (enabled ? "on" : "off"));
        });

    document
        .getElementById("toggleAbsolute")
        .addEventListener("change", function () {
            const enabled = this.checked;
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send("toggleAbsolute:" + (enabled ? "on" : "off"));
            }
        });

    document
        .getElementById("absoluteThreshold")
        .addEventListener("change", function () {
            const threshold = parseFloat(this.value);
            if (!isNaN(threshold) && ws && ws.readyState === WebSocket.OPEN) {
                ws.send("setAbsoluteThreshold:" + threshold.toFixed(2));
            }
        });

    document
        .getElementById("toggleKnownOffsetSpinCorrection")
        .addEventListener("change", function () {
            const enabled = this.checked;
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(
                    "toggleKnownOffsetSpinCorrection:" +
                        (enabled ? "on" : "off")
                );
            }
        });

    document
        .getElementById("knownOffset")
        .addEventListener("change", function () {
            const threshold = parseFloat(this.value);
            if (!isNaN(threshold) && ws && ws.readyState === WebSocket.OPEN) {
                ws.send("setKnownOffset:" + threshold.toFixed(2));
            }
        });

    document.getElementById("resetPayload").addEventListener("click", () => {
        ws.send("resetPayload");
    });

    // Initialize data arrays with 300 zeros
    let labels = Array(300).fill("");
    let accelDataX = Array(300).fill(0),
        accelDataY = Array(300).fill(0),
        accelDataZ = Array(300).fill(0),
        accelDataA = Array(300).fill(0);

    let gyroDataX = Array(300).fill(0),
        gyroDataY = Array(300).fill(0),
        gyroDataZ = Array(300).fill(0);

    let calibrationLossData = Array(300).fill(0);

    let rollData = Array(300).fill(0),
        pitchData = Array(300).fill(0),
        yawData = Array(300).fill(0);
    let quatDataQ0 = Array(300).fill(0),
        quatDataQ1 = Array(300).fill(0),
        quatDataQ2 = Array(300).fill(0),
        quatDataQ3 = Array(300).fill(0);
    let rMagnitudeData = Array(300).fill(0);
    let rAngleData = Array(300).fill(0);

    // Chart.js configuration
    Chart.defaults.color = "#4a5568";
    Chart.defaults.font.family =
        "-apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif";
    Chart.defaults.font.size = 9;

    // Accelerometer Chart
    const accelCtx = document.getElementById("accel_chart").getContext("2d");
    const accelChart = new Chart(accelCtx, {
        type: "line",
        data: {
            labels: labels,
            datasets: [
                {
                    label: "X Acceleration",
                    data: accelDataX,
                    borderColor: "#F56565",
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
                {
                    label: "Y Acceleration",
                    data: accelDataY,
                    borderColor: "#48BB78",
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
                {
                    label: "Z Acceleration",
                    data: accelDataZ,
                    borderColor: "#4299E1",
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
                {
                    label: "Absolute Acceleration",
                    data: accelDataA,
                    borderColor: "#2D3748",
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
            ],
        },
        options: {
            animation: false,
            responsive: true,
            maintainAspectRatio: false,
            interaction: {
                intersect: false,
                mode: "index",
            },
            plugins: {
                legend: {
                    position: "top",
                    labels: {
                        boxWidth: 10,
                        padding: 6,
                        font: {
                            size: 12,
                        },
                    },
                },
                tooltip: {
                    backgroundColor: "rgba(0, 0, 0, 0.7)",
                    bodyFont: {
                        size: 12,
                    },
                    padding: 6,
                },
            },
            scales: {
                x: {
                    display: false,
                    grid: {
                        display: false,
                    },
                },
                y: {
                    beginAtZero: false,
                    suggestedMin: -10,
                    suggestedMax: 10,
                    grid: {
                        color: "rgba(0, 0, 0, 0.05)",
                    },
                    ticks: {
                        font: {
                            size: 12,
                        },
                    },
                },
            },
        },
    });

    // Gyroscope Chart
    const gyroCtx = document.getElementById("gyro_chart").getContext("2d");
    const gyroChart = new Chart(gyroCtx, {
        type: "line",
        data: {
            labels: labels,
            datasets: [
                {
                    label: "X Rotation",
                    data: gyroDataX,
                    borderColor: "#F56565",
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
                {
                    label: "Y Rotation",
                    data: gyroDataY,
                    borderColor: "#48BB78",
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
                {
                    label: "Z Rotation",
                    data: gyroDataZ,
                    borderColor: "#4299E1",
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
            ],
        },
        options: {
            animation: false,
            responsive: true,
            maintainAspectRatio: false,
            interaction: {
                intersect: false,
                mode: "index",
            },
            plugins: {
                legend: {
                    position: "top",
                    labels: {
                        boxWidth: 10,
                        padding: 6,
                        font: {
                            size: 12,
                        },
                    },
                },
                tooltip: {
                    backgroundColor: "rgba(0, 0, 0, 0.7)",
                    bodyFont: {
                        size: 12,
                    },
                    padding: 6,
                },
            },
            scales: {
                x: {
                    display: false,
                    grid: {
                        display: false,
                    },
                },
                y: {
                    beginAtZero: false,
                    suggestedMin: -20,
                    suggestedMax: 20,
                    grid: {
                        color: "rgba(0, 0, 0, 0.05)",
                    },
                    ticks: {
                        font: {
                            size: 12,
                        },
                    },
                },
            },
        },
    });

    // Calibration Loss Chart
    const calibrationCtx = document
        .getElementById("calibration_chart")
        .getContext("2d");
    const calibrationChart = new Chart(calibrationCtx, {
        type: "line",
        data: {
            labels: labels,
            datasets: [
                {
                    label: "Calibration Loss",
                    data: calibrationLossData,
                    borderColor: "#805AD5",
                    backgroundColor: "rgba(128, 90, 213, 0.1)",
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: true,
                    tension: 0.4,
                },
            ],
        },
        options: {
            animation: false,
            responsive: true,
            maintainAspectRatio: false,
            interaction: {
                intersect: false,
                mode: "index",
            },
            plugins: {
                legend: {
                    position: "top",
                    labels: {
                        boxWidth: 10,
                        padding: 6,
                        font: {
                            size: 12,
                        },
                    },
                },
                tooltip: {
                    backgroundColor: "rgba(0, 0, 0, 0.7)",
                    bodyFont: {
                        size: 12,
                    },
                    padding: 6,
                },
            },
            scales: {
                x: {
                    display: false,
                    grid: {
                        display: false,
                    },
                },
                y: {
                    beginAtZero: true,
                    grid: {
                        color: "rgba(0, 0, 0, 0.05)",
                    },
                    ticks: {
                        font: {
                            size: 12,
                        },
                    },
                },
            },
        },
    });

    const quatCtx = document.getElementById("quat_chart").getContext("2d");
    const quatChart = new Chart(quatCtx, {
        type: "line",
        data: {
            labels: labels,
            datasets: [
                {
                    label: "q0",
                    data: quatDataQ0,
                    borderColor: "#ECC94B", // Yellow
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
                {
                    label: "q1",
                    data: quatDataQ1,
                    borderColor: "#F56565", // Red
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
                {
                    label: "q2",
                    data: quatDataQ2,
                    borderColor: "#48BB78", // Green
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
                {
                    label: "q3",
                    data: quatDataQ3,
                    borderColor: "#4299E1", // Blue
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
            ],
        },
        options: {
            animation: false,
            responsive: true,
            maintainAspectRatio: false,
            interaction: {
                intersect: false,
                mode: "index",
            },
            plugins: {
                legend: {
                    position: "top",
                    labels: {
                        boxWidth: 10,
                        padding: 6,
                        font: {
                            size: 12,
                        },
                    },
                },
                tooltip: {
                    backgroundColor: "rgba(0, 0, 0, 0.7)",
                    bodyFont: {
                        size: 12,
                    },
                    padding: 6,
                },
            },
            scales: {
                x: {
                    display: false,
                    grid: {
                        display: false,
                    },
                },
                y: {
                    beginAtZero: false,
                    suggestedMin: -1,
                    suggestedMax: 1,
                    grid: {
                        color: "rgba(0, 0, 0, 0.05)",
                    },
                    ticks: {
                        font: {
                            size: 12,
                        },
                    },
                },
            },
        },
    });

    const eulerCtx = document.getElementById("euler_chart").getContext("2d");

    const eulerChart = new Chart(eulerCtx, {
        type: "line",
        data: {
            labels: labels,
            datasets: [
                {
                    label: "Roll (°)",
                    data: rollData,
                    borderColor: "#F56565",
                    borderWidth: 2,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
                {
                    label: "Pitch (°)",
                    data: pitchData,
                    borderColor: "#48BB78",
                    borderWidth: 2,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
                {
                    label: "Yaw (°)",
                    data: yawData,
                    borderColor: "#4299E1",
                    borderWidth: 2,
                    pointRadius: 0,
                    fill: false,
                    tension: 0.1,
                },
            ],
        },
        options: {
            animation: false,
            responsive: true,
            maintainAspectRatio: false,
            interaction: {
                intersect: false,
                mode: "index",
            },
            plugins: {
                legend: {
                    position: "top",
                    labels: {
                        boxWidth: 12,
                        padding: 8,
                        font: {
                            size: 11,
                        },
                    },
                },
                tooltip: {
                    backgroundColor: "rgba(0, 0, 0, 0.7)",
                    bodyFont: {
                        size: 12,
                    },
                    padding: 8,
                },
            },
            scales: {
                x: {
                    display: false,
                    grid: {
                        display: false,
                    },
                },
                y: {
                    beginAtZero: false,
                    suggestedMin: -180,
                    suggestedMax: 180,
                    grid: {
                        color: "rgba(0, 0, 0, 0.05)",
                    },
                    ticks: {
                        font: {
                            size: 10,
                        },
                    },
                },
            },
        },
    });

    const rCtx = document.getElementById("r_polar_chart").getContext("2d");
    const rChart = new Chart(rCtx, {
        type: "polarArea",
        data: {
            labels: ["Offset r"],
            datasets: [
                {
                    label: "Magnitude of r",
                    data: [0],
                    backgroundColor: ["rgba(66, 153, 225, 0.5)"],
                    borderColor: ["rgba(66, 153, 225, 1)"],
                    borderWidth: 1,
                },
            ],
        },
        options: {
            animation: false,
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                r: {
                    suggestedMin: 0,
                    suggestedMax: 0.1,
                    ticks: {
                        stepSize: 0.01,
                    },
                },
            },
            plugins: {
                legend: { display: false },
                tooltip: {
                    callbacks: {
                        label: function (context) {
                            const angle = rAngleData[rAngleData.length - 1];
                            return `r = ${context.raw.toFixed(
                                3
                            )} m @ ${angle.toFixed(1)}°`;
                        },
                    },
                },
            },
        },
    });

    // Resize charts when window size changes
    window.addEventListener("resize", function () {
        accelChart.resize();
        gyroChart.resize();
        calibrationChart.resize();
        quatChart.resize();
        eulerChart.resize();
    });

    // Button event handlers
    document.getElementById("resetFall").addEventListener("click", () => {
        ws.send("resetFall");
    });

    document.getElementById("deployParachute").addEventListener("click", () => {
        ws.send("deployParachute");
    });

    document.getElementById("deployPayload").addEventListener("click", () => {
        ws.send("deployPayload");
    });

    document.getElementById("recalibrate").addEventListener("click", () => {
        ws.send("recalibrate");
    });

    document.getElementById("downloadData").addEventListener("click", () => {
        window.location.href = "/downloadData";
    });
});
