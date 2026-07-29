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
      { timeout: 30_000 },
    );
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

    const screenshot = path.join(outputDir, `${viewport.name}.png`);
    await page.screenshot({ path: screenshot, fullPage: true });
    if (errors.length) {
      throw new Error(`${viewport.name}: browser errors: ${errors.join(" | ")}`);
    }

    results.push({
      viewport,
      telemetry,
      canvasPixels,
      screenshot,
    });
    await page.close();
  }
} finally {
  await browser.close();
}

console.log(JSON.stringify(results, null, 2));
