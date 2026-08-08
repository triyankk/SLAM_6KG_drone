import { chromium } from "playwright";
import fs from "node:fs/promises";
import path from "node:path";

const baseUrl =
  process.env.SLAM_POC_URL ?? "http://127.0.0.1:5173/slam-poc.html";
const outputDir = path.resolve("test-output");
await fs.mkdir(outputDir, { recursive: true });

function makeState() {
  return {
    schema_version: 1,
    session: "20260731T150000Z_slam-poc",
    elapsed_s: 18.4,
    ready_for_motion: false,
    map_sequence: 0,
    rgbd_path: [
      [0, 0, 0],
      [0.08, 0.01, 0],
      [0.18, 0.01, 0.01],
    ],
    rgbd: {
      connected: true,
      tracking: true,
      measured_fps: 29.8,
      tracking_success_ratio: 0.96,
      gyro_prior_coverage_ratio: 0.91,
      map_points: 18420,
      map_keyframes: 8,
      path_length_m: 0.18,
      compute_ms: 11.2,
      valid_depth_fraction: 0.84,
      position_local_flu_m: [0.18, 0.01, 0.01],
    },
    lio: {
      publishing: true,
      synchronized: true,
      rows: 412,
      path_length_m: 0.19,
      path: [
        [0, 0, 0],
        [0.09, 0.01, 0],
        [0.19, 0.01, 0.01],
      ],
    },
    imu: { connected: true, rate_hz: 200.1, clock_ready: true, queue_drops: 0 },
    lidar: { connected: true, rate_hz: 5.0, clock_ready: true, queue_drops: 0 },
    guide: {
      phase: "synchronizing",
      sequence: 0,
      label: "PREPARING",
      instruction: "SENSORS SYNCHRONIZING",
      detail: "Keep the aircraft disarmed and still.",
      progress: 0.8,
      hold_remaining_s: null,
      rgbd_horizontal_m: 0,
      lio_horizontal_m: 0,
      rgbd_vertical_m: 0,
      lio_vertical_m: 0,
      target_m: 0.3,
      estimator_gap_m: 0,
      vertical_warning: false,
      complete: false,
    },
    rtl_shadow: {
      state: "waiting_for_launch",
      shadow_only: true,
      pose_sent_to_cube: false,
      velocity_sent_to_cube: false,
      launch_captured: false,
      breadcrumbs: 0,
      latest: null,
    },
  };
}

const emptyMap = {
  sequence: 0,
  encoding: "int16_le_base64",
  scale_m: 0.01,
  point_count: 0,
  points_b64: "",
  colors_b64: "",
};

const browser = await chromium.launch({ headless: true });
const viewports = [
  { name: "desktop", width: 1440, height: 900 },
  { name: "mobile", width: 390, height: 844 },
];

try {
  for (const viewport of viewports) {
    const state = makeState();
    const page = await browser.newPage({ viewport });
    const errors = [];
    page.on("console", (message) => {
      if (message.type() === "error") errors.push(message.text());
    });
    page.on("pageerror", (error) => errors.push(error.message));
    await page.route("**/api/slam-poc/map", async (route) => {
      await route.fulfill({
        contentType: "application/json",
        body: JSON.stringify(emptyMap),
      });
    });
    await page.route("**/api/slam-poc", async (route) => {
      await route.fulfill({
        contentType: "application/json",
        body: JSON.stringify(state),
      });
    });
    await page.goto(baseUrl, { waitUntil: "domcontentloaded" });
    await page.waitForFunction(
      () =>
        document.querySelector("#guideInstruction")?.textContent ===
        "SENSORS SYNCHRONIZING",
    );

    state.ready_for_motion = true;
    state.guide = {
      ...state.guide,
      phase: "outbound",
      sequence: 2,
      label: "STEP 2 OF 5",
      instruction: "MOVE HORIZONTALLY",
      detail: "Move slowly in one clear direction; keep height and yaw steady.",
      progress: 0.63,
      rgbd_horizontal_m: 0.2,
      lio_horizontal_m: 0.19,
    };
    state.rtl_shadow = {
      ...state.rtl_shadow,
      state: "returning",
      launch_captured: true,
      breadcrumbs: 5,
      latest: {
        position_local_flu_m: [0.2, 0.01, 0],
        target_local_flu_m: [0.1, 0, 0],
        proposed_speed_mps: 0.16,
      },
    };
    await page.waitForSelector("#instructionFlash.is-flashing");
    const flashCheck = await page.evaluate(() => {
      const flash = document.querySelector("#instructionFlash");
      const animation = flash.getAnimations()[0];
      const keyframes = animation?.effect?.getKeyframes() ?? [];
      return {
        instructionDuringFlash: document.querySelector("#guideInstruction")?.textContent,
        animationName: getComputedStyle(flash).animationName,
        brightPeaks: keyframes.filter(
          (frame) => Number.parseFloat(frame.opacity ?? "0") > 0.5,
        ).length,
      };
    });
    if (
      flashCheck.instructionDuringFlash !== "SENSORS SYNCHRONIZING" ||
      flashCheck.animationName !== "instruction-double-flash" ||
      flashCheck.brightPeaks !== 2
    ) {
      throw new Error(
        `${viewport.name}: double-flash gate failed: ${JSON.stringify(flashCheck)}`,
      );
    }

    await page.waitForFunction(
      () =>
        document.querySelector("#guideInstruction")?.textContent ===
        "MOVE HORIZONTALLY",
    );
    await page.waitForTimeout(150);
    const result = await page.evaluate(() => {
      const banner = document.querySelector("#guideBanner").getBoundingClientRect();
      const status = document.querySelector(".scene-status").getBoundingClientRect();
      const legend = document.querySelector(".legend").getBoundingClientRect();
      const scene = document.querySelector(".scene-panel").getBoundingClientRect();
      const progress = document.querySelector("#guideProgress").getBoundingClientRect();
      return {
        instruction: document.querySelector("#guideInstruction")?.textContent,
        guideValue: document.querySelector("#guideValue")?.textContent,
        rtlState: document.querySelector("#rtlState")?.textContent,
        rtlCommand: document.querySelector("#rtlCommand")?.textContent,
        progressWidth: progress.width,
        bannerTop: banner.top,
        bannerBottom: banner.bottom,
        bannerLeft: banner.left,
        bannerRight: banner.right,
        statusBottom: status.bottom,
        legendTop: legend.top,
        sceneLeft: scene.left,
        sceneRight: scene.right,
        bodyWidth: document.body.scrollWidth,
        viewportWidth: window.innerWidth,
      };
    });
    if (errors.length) {
      throw new Error(`${viewport.name}: ${errors.join(" | ")}`);
    }
    if (
      result.instruction !== "MOVE HORIZONTALLY" ||
      result.guideValue !== "0.19 / 0.30 m" ||
      result.rtlState !== "RETURNING" ||
      result.rtlCommand !== "0.16 m/s" ||
      result.progressWidth < 40
    ) {
      throw new Error(
        `${viewport.name}: guide telemetry did not render: ${JSON.stringify(result)}`,
      );
    }
    if (
      result.bannerTop < result.statusBottom + 4 ||
      result.bannerBottom > result.legendTop - 4 ||
      result.bannerLeft < result.sceneLeft ||
      result.bannerRight > result.sceneRight ||
      result.bodyWidth > result.viewportWidth + 1
    ) {
      throw new Error(
        `${viewport.name}: guide overlaps or overflows: ${JSON.stringify(result)}`,
      );
    }
    await page.screenshot({
      path: path.join(outputDir, `${viewport.name}-slam-poc-guide.png`),
      fullPage: true,
    });
    await page.locator("#slamScene").screenshot({
      path: path.join(outputDir, `${viewport.name}-slam-poc-canvas.png`),
    });
    await page.close();
  }
} finally {
  await browser.close();
}
