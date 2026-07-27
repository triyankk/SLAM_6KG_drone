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

    await page.goto(baseUrl, { waitUntil: "networkidle" });
    await page.waitForFunction(
      () => document.querySelector("#sourceBadge")?.textContent === "DEMO",
      null,
      { timeout: 10_000 },
    );
    await page.waitForTimeout(1200);

    const telemetry = await page.evaluate(() => ({
      flowX: document.querySelector("#flowX")?.textContent,
      flowY: document.querySelector("#flowY")?.textContent,
      range: document.querySelector("#rangeValue")?.textContent,
      quality: document.querySelector("#qualityValue")?.textContent,
      accelZ: document.querySelector("#accelZ")?.textContent,
      gyroZ: document.querySelector("#gyroZ")?.textContent,
      imuSource: document.querySelector("#imuSource")?.textContent,
    }));
    if (
      telemetry.flowX === "0.000" &&
      telemetry.flowY === "0.000"
    ) {
      throw new Error(`${viewport.name}: demo flow did not update`);
    }
    if (
      telemetry.accelZ === "0.00" ||
      !telemetry.imuSource?.includes("DEMO_IMU")
    ) {
      throw new Error(`${viewport.name}: demo IMU did not update`);
    }

    const layout = await page.evaluate(() => {
      const selectors = [
        ".topbar",
        ".telemetry-panel",
        ".controlbar",
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
      const overlap =
        controls.left < telemetryPanel.right &&
        controls.right > telemetryPanel.left &&
        controls.top < telemetryPanel.bottom &&
        controls.bottom > telemetryPanel.top;
      const outOfViewport = Object.entries(rectangles)
        .filter(([selector]) => selector !== "#scene")
        .filter(([, rect]) =>
          rect.left < -1 ||
          rect.top < -1 ||
          rect.right > window.innerWidth + 1 ||
          rect.bottom > window.innerHeight + 1
        )
        .map(([selector]) => selector);
      return { rectangles, overlap, outOfViewport };
    });
    if (layout.overlap) {
      throw new Error(`${viewport.name}: control bar overlaps telemetry`);
    }
    if (layout.outOfViewport.length) {
      throw new Error(
        `${viewport.name}: out-of-viewport UI: ${layout.outOfViewport.join(", ")}`,
      );
    }

    const canvasPixels = await page.evaluate(() => {
      const canvas = document.querySelector("#scene");
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
          Math.abs(pixels[index] - 17) +
          Math.abs(pixels[index + 1] - 19) +
          Math.abs(pixels[index + 2] - 18);
        if (difference > 14) changedPixels += 1;
      }
      return {
        available: true,
        changedPixels,
        totalPixels: canvas.width * canvas.height,
      };
    });
    if (!canvasPixels.available || canvasPixels.changedPixels < 5000) {
      throw new Error(
        `${viewport.name}: WebGL canvas appears blank: ${JSON.stringify(canvasPixels)}`,
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
