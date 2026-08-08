import { chromium } from "playwright";
import fs from "node:fs/promises";
import path from "node:path";

const baseUrl = process.env.VISUALIZER_URL ?? "http://127.0.0.1:8765";
const expectedSource = process.env.EXPECTED_SOURCE ?? "UART";
const outputDir = path.resolve("test-output");
await fs.mkdir(outputDir, { recursive: true });

const browser = await chromium.launch({
  headless: true,
  args: ["--enable-webgl", "--use-gl=angle", "--use-angle=swiftshader"],
});

try {
  const page = await browser.newPage({
    viewport: { width: 1440, height: 900 },
    deviceScaleFactor: 1,
  });
  const errors = [];
  page.on("console", (message) => {
    if (message.type() === "error") errors.push(message.text());
  });
  page.on("pageerror", (error) => errors.push(error.message));

  await page.goto(baseUrl, { waitUntil: "domcontentloaded" });
  await page.waitForFunction(
    (source) =>
      document.querySelector("#sourceBadge")?.textContent === source &&
      document.querySelector("#linkText")?.textContent === "Live",
    expectedSource,
    { timeout: 120_000 },
  );
  await page.locator("#spatialViewButton").click();
  await page.waitForFunction(
    () =>
      document.querySelector("#lidarCloudStatus")?.textContent === "LIVE" &&
      document.querySelector("#depthCloudStatus")?.textContent === "LIVE" &&
      document.querySelector("#mapPoints")?.textContent !== "0" &&
      document.querySelector("#clearanceStatus")?.textContent !== "UNKNOWN" &&
      Number(document.querySelector("#spatialView")?.dataset.trajectoryPoints) >
        1,
    null,
    { timeout: 120_000 },
  );
  await page.waitForTimeout(1500);

  const summary = await page.evaluate(() => ({
    lidar: {
      status: document.querySelector("#lidarCloudStatus")?.textContent,
      points: document.querySelector("#lidarCloudPoints")?.textContent,
      rate: document.querySelector("#lidarCloudRate")?.textContent,
      age: document.querySelector("#lidarCloudAge")?.textContent,
    },
    depth: {
      status: document.querySelector("#depthCloudStatus")?.textContent,
      points: document.querySelector("#depthCloudPoints")?.textContent,
      rate: document.querySelector("#depthCloudRate")?.textContent,
      age: document.querySelector("#depthCloudAge")?.textContent,
    },
    mapPoints: document.querySelector("#mapPoints")?.textContent,
    mapFrames: document.querySelector("#mapFrames")?.textContent,
    clearance: {
      status: document.querySelector("#clearanceStatus")?.textContent,
      limit: document.querySelector("#clearanceLimit")?.textContent,
      nearest: document.querySelector("#clearanceNearest")?.textContent,
      margin: document.querySelector("#clearanceMargin")?.textContent,
    },
    pose: {
      status: document.querySelector("#poseStatus")?.textContent,
      x: document.querySelector("#poseX")?.textContent,
      y: document.querySelector("#poseY")?.textContent,
      z: document.querySelector("#poseZ")?.textContent,
      speed: document.querySelector("#poseSpeed")?.textContent,
    },
    trajectory: {
      enabled:
        document.querySelector("#mapTrajectoryButton")?.getAttribute(
          "aria-pressed",
        ),
      points: Number(
        document.querySelector("#spatialView")?.dataset.trajectoryPoints,
      ),
      lioPathM: Number(document.querySelector("#spatialView")?.dataset.lioPathM),
      rgbdPathM: Number(
        document.querySelector("#spatialView")?.dataset.rgbdPathM,
      ),
      detail: document.querySelector("#trajectoryDetail")?.textContent,
    },
    defaults: {
      view: document.querySelector("#app")?.dataset.view,
      depthVisible: document.querySelector("#depthVisibility")?.checked,
      lidarVisible: document.querySelector("#lidarVisibility")?.checked,
    },
  }));
  if (errors.length) {
    throw new Error(`browser errors: ${errors.join(" | ")}`);
  }
  if (
    summary.clearance.limit !== "1.50 m" ||
    !["CLEAR", "BREACH"].includes(summary.clearance.status)
  ) {
    throw new Error(
      `clearance telemetry is invalid: ${JSON.stringify(summary.clearance)}`,
    );
  }
  if (
    summary.defaults.view !== "spatial" ||
    !summary.defaults.depthVisible ||
    summary.defaults.lidarVisible ||
    summary.trajectory.enabled !== "true" ||
    summary.trajectory.points <= 1
  ) {
    throw new Error(
      `3D startup or trajectory state is invalid: ${JSON.stringify({
        defaults: summary.defaults,
        trajectory: summary.trajectory,
      })}`,
    );
  }
  await page.locator("#lidarVisibility").check();
  await page.locator("#depthVisibility").uncheck();
  if (
    !(await page.locator("#lidarVisibility").isChecked()) ||
    (await page.locator("#depthVisibility").isChecked())
  ) {
    throw new Error("live sensor visibility toggles failed");
  }
  await page.locator("#depthVisibility").check();
  await page.locator("#lidarVisibility").uncheck();
  const mapPixels = await page.evaluate(() => {
    const canvas = document.querySelector("#mapScene");
    const gl =
      canvas.getContext("webgl2", { preserveDrawingBuffer: true }) ||
      canvas.getContext("webgl", { preserveDrawingBuffer: true });
    if (!gl) return { available: false, changedPixels: 0 };
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
    return { available: true, changedPixels };
  });
  if (!mapPixels.available || mapPixels.changedPixels < 5000) {
    throw new Error(`spatial canvas appears blank: ${JSON.stringify(mapPixels)}`);
  }
  const screenshot = path.join(outputDir, "live-spatial.png");
  await page.screenshot({ path: screenshot, fullPage: true });
  console.log(JSON.stringify({ summary, mapPixels, screenshot }, null, 2));
} finally {
  await browser.close();
}
