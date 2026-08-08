import { chromium } from "playwright";
import fs from "node:fs/promises";
import path from "node:path";

const baseUrl = process.env.VISUALIZER_URL ?? "http://127.0.0.1:8765";
const outputDir = path.resolve("test-output");
await fs.mkdir(outputDir, { recursive: true });

const browser = await chromium.launch({
  headless: true,
  args: ["--enable-webgl", "--use-gl=angle", "--use-angle=swiftshader"],
});

const viewports = [
  { name: "desktop", width: 1440, height: 900 },
  { name: "mobile", width: 390, height: 844 },
];
const results = [];

try {
  for (const viewport of viewports) {
    const page = await browser.newPage({
      viewport: { width: viewport.width, height: viewport.height },
      deviceScaleFactor: 1,
    });
    const errors = [];
    page.on("console", (message) => {
      if (message.type() === "error") errors.push(message.text());
    });
    page.on("pageerror", (error) => errors.push(error.message));

    await page.goto(baseUrl, { waitUntil: "domcontentloaded" });
    await page.waitForFunction(
      () => document.querySelector("#sourceBadge")?.textContent === "DEMO",
      null,
      { timeout: 120_000 },
    );
    await page.waitForFunction(
      () =>
        Number(
          document.querySelector("#spatialView")?.dataset.trajectoryPoints,
        ) > 100,
      null,
      { timeout: 120_000 },
    );
    const startup = await page.evaluate(() => ({
      view: document.querySelector("#app")?.dataset.view,
      trajectoryOn:
        document.querySelector("#mapTrajectoryButton")?.getAttribute(
          "aria-pressed",
        ),
      depthVisible: document.querySelector("#depthVisibility")?.checked,
      lidarVisible: document.querySelector("#lidarVisibility")?.checked,
      trajectoryPoints: Number(
        document.querySelector("#spatialView")?.dataset.trajectoryPoints,
      ),
      lioPathM: Number(document.querySelector("#spatialView")?.dataset.lioPathM),
    }));
    if (
      startup.view !== "spatial" ||
      startup.trajectoryOn !== "true" ||
      !startup.depthVisible ||
      startup.lidarVisible ||
      startup.trajectoryPoints <= 100 ||
      startup.lioPathM <= 1
    ) {
      throw new Error(
        `${viewport.name}: invalid default 3D state ${JSON.stringify(startup)}`,
      );
    }
    const cueResponse = await page.evaluate(async () => {
      const response = await fetch("/api/cue", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          message: "MOVE DRONE NOW",
          detail: "Translate slowly and keep level.",
          flash_count: 2,
          duration_s: 1,
        }),
      });
      return { ok: response.ok, status: response.status };
    });
    if (!cueResponse.ok || cueResponse.status !== 202) {
      throw new Error(
        `${viewport.name}: visual cue request failed ${JSON.stringify(cueResponse)}`,
      );
    }
    await page.waitForFunction(
      () =>
        !document.querySelector("#visualCue")?.hidden &&
        document.querySelector("#visualCueMessage")?.textContent ===
          "MOVE DRONE NOW",
      null,
      { timeout: 30_000 },
    );
    await page.waitForFunction(
      () => document.querySelector("#visualCue")?.hidden,
      null,
      { timeout: 30_000 },
    );
    await page.locator("#motionViewButton").click();
    await page.waitForTimeout(1200);

    const telemetry = await page.evaluate(() => ({
      flowX: document.querySelector("#flowX")?.textContent,
      flowY: document.querySelector("#flowY")?.textContent,
      flowUnit: document.querySelector(".flow-axis .unit")?.textContent,
      flowAngularRate: document.querySelector("#flowAngularRate")?.textContent,
      range: document.querySelector("#rangeValue")?.textContent,
      quality: document.querySelector("#qualityValue")?.textContent,
      accelZ: document.querySelector("#accelZ")?.textContent,
      gyroZ: document.querySelector("#gyroZ")?.textContent,
      imuSource: document.querySelector("#imuSource")?.textContent,
      rosStatus: document.querySelector("#rosImuStatus")?.textContent,
      rosRate: document.querySelector("#rosImuRate")?.textContent,
      rosRoll: document.querySelector("#rosRollValue")?.textContent,
      rosFrameStatus: document.querySelector("#rosFrameStatus")?.textContent,
      alignImuDisabled: document.querySelector("#alignImuButton")?.disabled,
      cubeMount: document.querySelector("#cubeMountValue")?.textContent,
    }));
    if (telemetry.flowX === "0.000" && telemetry.flowY === "0.000") {
      throw new Error(`${viewport.name}: demo flow did not update`);
    }
    if (
      telemetry.flowUnit !== "m/s" ||
      !telemetry.flowAngularRate?.includes("rad/s")
    ) {
      throw new Error(`${viewport.name}: flow units are not explicit`);
    }
    if (
      telemetry.accelZ === "0.00" ||
      !telemetry.imuSource?.includes("DEMO_IMU")
    ) {
      throw new Error(`${viewport.name}: demo IMU did not update`);
    }
    if (
      telemetry.rosStatus !== "LIVE" ||
      telemetry.rosRate !== "40.0 Hz" ||
      telemetry.rosRoll === "0.0" ||
      telemetry.rosFrameStatus !== "X/-Y/-Z REF - MEASURED" ||
      telemetry.alignImuDisabled
    ) {
      throw new Error(`${viewport.name}: ROS IMU inset did not update`);
    }
    if (!telemetry.cubeMount?.includes("Yaw270")) {
      throw new Error(`${viewport.name}: Cube mount is not visible`);
    }

    const layout = await page.evaluate(() => {
      const selectors = [
        ".topbar",
        ".telemetry-panel",
        ".controlbar",
        ".ros-imu-panel",
        ".trace-panel",
        "#scene",
      ];
      const rectangles = Object.fromEntries(
        selectors.map((selector) => {
          const rect = document.querySelector(selector).getBoundingClientRect();
          return [
            selector,
            {
              left: rect.left,
              top: rect.top,
              right: rect.right,
              bottom: rect.bottom,
              width: rect.width,
              height: rect.height,
            },
          ];
        }),
      );
      const controls = rectangles[".controlbar"];
      const telemetryPanel = rectangles[".telemetry-panel"];
      const rosImuPanel = rectangles[".ros-imu-panel"];
      const topbar = rectangles[".topbar"];
      const overlaps = (first, second) =>
        first.width > 0 &&
        first.height > 0 &&
        second.width > 0 &&
        second.height > 0 &&
        first.left < second.right &&
        first.right > second.left &&
        first.top < second.bottom &&
        first.bottom > second.top;
      const overlap =
        overlaps(controls, telemetryPanel) ||
        overlaps(rosImuPanel, controls) ||
        overlaps(rosImuPanel, telemetryPanel) ||
        overlaps(rosImuPanel, topbar);
      const outOfViewport = Object.entries(rectangles)
        .filter(([selector]) => selector !== "#scene")
        .filter(([, rect]) =>
          rect.left < -1 ||
          rect.top < -1 ||
          rect.right > window.innerWidth + 1 ||
          rect.bottom > window.innerHeight + 1
        )
        .map(([selector]) => selector);
      const clippedControls = Array.from(
        document.querySelectorAll(".controlbar > *"),
      )
        .filter((element) => {
          const rect = element.getBoundingClientRect();
          return (
            rect.left < controls.left - 1 ||
            rect.right > controls.right + 1 ||
            rect.top < controls.top - 1 ||
            rect.bottom > controls.bottom + 1
          );
        })
        .map((element) => element.id || element.className);
      return { rectangles, overlap, outOfViewport, clippedControls };
    });
    if (layout.overlap) {
      throw new Error(`${viewport.name}: control bar overlaps telemetry`);
    }
    if (layout.outOfViewport.length) {
      throw new Error(
        `${viewport.name}: out-of-viewport UI: ${layout.outOfViewport.join(", ")}`,
      );
    }
    if (layout.clippedControls.length) {
      throw new Error(
        `${viewport.name}: clipped controls: ${layout.clippedControls.join(", ")}`,
      );
    }

    const canvasPixels = await page.evaluate(() => {
      function inspect(selector, background) {
        const canvas = document.querySelector(selector);
        const gl =
          canvas.getContext("webgl2", { preserveDrawingBuffer: true }) ||
          canvas.getContext("webgl", { preserveDrawingBuffer: true });
        if (!gl) {
          return { available: false, changedPixels: 0, totalPixels: 0 };
        }
        const pixels = new Uint8Array(canvas.width * canvas.height * 4);
        gl.readPixels(
          0,
          0,
          canvas.width,
          canvas.height,
          gl.RGBA,
          gl.UNSIGNED_BYTE,
          pixels,
        );
        let changedPixels = 0;
        for (let index = 0; index < pixels.length; index += 4) {
          const difference =
            Math.abs(pixels[index] - background[0]) +
            Math.abs(pixels[index + 1] - background[1]) +
            Math.abs(pixels[index + 2] - background[2]);
          if (difference > 14) changedPixels += 1;
        }
        return {
          available: true,
          changedPixels,
          totalPixels: canvas.width * canvas.height,
        };
      }
      return {
        main: inspect("#scene", [17, 19, 18]),
        rosImu: inspect("#rosImuScene", [23, 26, 24]),
      };
    });
    if (
      !canvasPixels.main.available ||
      canvasPixels.main.changedPixels < 5000
    ) {
      throw new Error(
        `${viewport.name}: WebGL canvas appears blank: ${JSON.stringify(canvasPixels)}`,
      );
    }
    if (
      !canvasPixels.rosImu.available ||
      canvasPixels.rosImu.changedPixels < 300
    ) {
      throw new Error(
        `${viewport.name}: ROS IMU canvas appears blank: ${JSON.stringify(canvasPixels)}`,
      );
    }

    await page.locator("#topButton").click();
    await page.locator("#pauseButton").click();
    await page.waitForTimeout(120);
    if (!(await page.locator("#displayState").isVisible())) {
      throw new Error(`${viewport.name}: pause state did not appear`);
    }
    await page.locator("#pauseButton").click();
    await page.locator("#perspectiveButton").click();
    await page.locator("#alignImuButton").click();
    if (
      !(await page.locator("#alignImuButton").evaluate((button) =>
        button.classList.contains("is-active"),
      ))
    ) {
      throw new Error(`${viewport.name}: IMU reference alignment did not run`);
    }

    const screenshot = path.join(outputDir, `${viewport.name}-motion.png`);
    await page.screenshot({ path: screenshot, fullPage: true });

    await page.locator("#spatialViewButton").click();
    await page.waitForFunction(
      () =>
        document.querySelector("#lidarCloudStatus")?.textContent === "LIVE" &&
        document.querySelector("#depthCloudStatus")?.textContent === "LIVE" &&
        document.querySelector("#mapPoints")?.textContent !== "0",
      null,
      { timeout: 120_000 },
    );
    await page.waitForTimeout(800);

    const spatialTelemetry = await page.evaluate(() => ({
      lidarStatus: document.querySelector("#lidarCloudStatus")?.textContent,
      lidarPoints: document.querySelector("#lidarCloudPoints")?.textContent,
      depthStatus: document.querySelector("#depthCloudStatus")?.textContent,
      depthPoints: document.querySelector("#depthCloudPoints")?.textContent,
      mapPoints: document.querySelector("#mapPoints")?.textContent,
      poseStatus: document.querySelector("#poseStatus")?.textContent,
      frameBadge: document.querySelector("#spatialFrameBadge")?.textContent,
      slamStatus: document.querySelector("#slamStatusBadge")?.textContent,
      trajectoryPressed:
        document.querySelector("#mapTrajectoryButton")?.getAttribute(
          "aria-pressed",
        ),
      clearanceLimit: document.querySelector("#clearanceLimit")?.textContent,
      trajectoryPoints: Number(
        document.querySelector("#spatialView")?.dataset.trajectoryPoints,
      ),
    }));
    if (
      spatialTelemetry.lidarStatus !== "LIVE" ||
      spatialTelemetry.depthStatus !== "LIVE" ||
      spatialTelemetry.mapPoints === "0" ||
      spatialTelemetry.poseStatus !== "CUBE LOCAL LIVE" ||
      spatialTelemetry.frameBadge !== "ROLLING LOCAL SCAN" ||
      spatialTelemetry.slamStatus !== "SLAM MONITOR" ||
      spatialTelemetry.trajectoryPressed !== "true" ||
      spatialTelemetry.clearanceLimit !== "1.50 m" ||
      spatialTelemetry.trajectoryPoints <= 100
    ) {
      throw new Error(
        `${viewport.name}: spatial telemetry incomplete: ${JSON.stringify(spatialTelemetry)}`,
      );
    }

    const spatialLayout = await page.evaluate(() => {
      const selectors = [
        ".topbar",
        ".spatial-sidebar",
        ".spatial-controlbar",
        ".spatial-status-strip",
        "#mapScene",
      ];
      const rectangles = Object.fromEntries(
        selectors.map((selector) => {
          const rect = document.querySelector(selector).getBoundingClientRect();
          return [
            selector,
            {
              left: rect.left,
              top: rect.top,
              right: rect.right,
              bottom: rect.bottom,
              width: rect.width,
              height: rect.height,
            },
          ];
        }),
      );
      const overlaps = (first, second) =>
        first.width > 0 &&
        first.height > 0 &&
        second.width > 0 &&
        second.height > 0 &&
        first.left < second.right &&
        first.right > second.left &&
        first.top < second.bottom &&
        first.bottom > second.top;
      const overlap =
        overlaps(
          rectangles[".spatial-controlbar"],
          rectangles[".spatial-sidebar"],
        ) ||
        overlaps(
          rectangles[".spatial-status-strip"],
          rectangles[".topbar"],
        );
      const outOfViewport = Object.entries(rectangles)
        .filter(([selector]) => selector !== "#mapScene")
        .filter(
          ([, rect]) =>
            rect.left < -1 ||
            rect.top < -1 ||
            rect.right > window.innerWidth + 1 ||
            rect.bottom > window.innerHeight + 1,
        )
        .map(([selector]) => selector);
      const controls = rectangles[".spatial-controlbar"];
      const clippedControls = Array.from(
        document.querySelectorAll(".spatial-controlbar > *"),
      )
        .filter((element) => {
          const rect = element.getBoundingClientRect();
          return (
            rect.left < controls.left - 1 ||
            rect.right > controls.right + 1 ||
            rect.top < controls.top - 1 ||
            rect.bottom > controls.bottom + 1
          );
        })
        .map((element) => element.id || element.className);
      return { rectangles, overlap, outOfViewport, clippedControls };
    });
    if (spatialLayout.overlap) {
      throw new Error(`${viewport.name}: spatial controls overlap`);
    }
    if (spatialLayout.outOfViewport.length) {
      throw new Error(
        `${viewport.name}: spatial UI out of viewport: ${spatialLayout.outOfViewport.join(", ")}`,
      );
    }
    if (spatialLayout.clippedControls.length) {
      throw new Error(
        `${viewport.name}: clipped spatial controls: ${spatialLayout.clippedControls.join(", ")}`,
      );
    }

    const mapPixels = await page.evaluate(() => {
      const canvas = document.querySelector("#mapScene");
      const gl =
        canvas.getContext("webgl2", { preserveDrawingBuffer: true }) ||
        canvas.getContext("webgl", { preserveDrawingBuffer: true });
      if (!gl) return { available: false, changedPixels: 0, totalPixels: 0 };
      const pixels = new Uint8Array(canvas.width * canvas.height * 4);
      gl.readPixels(
        0,
        0,
        canvas.width,
        canvas.height,
        gl.RGBA,
        gl.UNSIGNED_BYTE,
        pixels,
      );
      let changedPixels = 0;
      for (let index = 0; index < pixels.length; index += 4) {
        const difference =
          Math.abs(pixels[index] - 12) +
          Math.abs(pixels[index + 1] - 15) +
          Math.abs(pixels[index + 2] - 16);
        if (difference > 14) changedPixels += 1;
      }
      return {
        available: true,
        changedPixels,
        totalPixels: canvas.width * canvas.height,
      };
    });
    if (!mapPixels.available || mapPixels.changedPixels < 5000) {
      throw new Error(
        `${viewport.name}: spatial WebGL canvas appears blank: ${JSON.stringify(mapPixels)}`,
      );
    }

    await page.locator("#mapTopButton").click();
    await page.locator("#mapFollowButton").click();
    await page.locator("#mapTrailButton").click();
    await page.locator("#lidarVisibility").check();
    await page.locator("#depthVisibility").uncheck();
    if (
      !(await page.locator("#lidarVisibility").isChecked()) ||
      (await page.locator("#depthVisibility").isChecked())
    ) {
      throw new Error(`${viewport.name}: sensor visibility toggles failed`);
    }
    await page.locator("#depthVisibility").check();
    await page.locator("#lidarVisibility").uncheck();
    await page.locator("#mapTrajectoryButton").click();
    if (
      (await page.locator("#mapTrajectoryButton").getAttribute("aria-pressed")) !==
      "false"
    ) {
      throw new Error(`${viewport.name}: trajectory toggle did not switch off`);
    }
    await page.locator("#mapTrajectoryButton").click();
    await page.locator("#mapClearButton").click();
    await page.locator("#mapResetButton").click();

    const spatialScreenshot = path.join(
      outputDir,
      `${viewport.name}-spatial.png`,
    );
    await page.screenshot({ path: spatialScreenshot, fullPage: true });
    await page.locator("#motionViewButton").click();
    if (errors.length) {
      throw new Error(`${viewport.name}: browser errors: ${errors.join(" | ")}`);
    }

    results.push({
      viewport,
      telemetry,
      spatialTelemetry,
      canvasPixels,
      mapPixels,
      screenshot,
      spatialScreenshot,
    });
    await page.close();
  }
} finally {
  await browser.close();
}

console.log(JSON.stringify(results, null, 2));
