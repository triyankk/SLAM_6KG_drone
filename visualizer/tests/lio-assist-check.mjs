import { chromium } from "playwright";
import fs from "node:fs/promises";
import path from "node:path";

const baseUrl =
  process.env.LIO_ASSIST_URL ?? "http://127.0.0.1:4173/lio-assist.html";
const outputDir = path.resolve("test-output");
await fs.mkdir(outputDir, { recursive: true });

const state = {
  schema_version: 1,
  session: "20260731T120000Z_lio-shadow",
  guide_kind: "translation",
  guide_phases: [
    { id: "settle", start_s: 0, end_s: 15, timeline_label: "STILL" },
    { id: "forward_1", start_s: 15, end_s: 35, timeline_label: "FWD" },
    { id: "center_1", start_s: 35, end_s: 55, timeline_label: "CENTER" },
    { id: "right_1", start_s: 55, end_s: 75, timeline_label: "RIGHT" },
    { id: "center_2", start_s: 75, end_s: 95, timeline_label: "CENTER" },
    { id: "final_still", start_s: 95, end_s: 110, timeline_label: "STILL" },
  ],
  ready: true,
  synchronized: true,
  publishing: true,
  guide_started: true,
  guide_complete: false,
  stop_requested: false,
  elapsed_s: 20.2,
  duration_s: 110,
  progress: 20.2 / 110,
  phase: {
    id: "forward_1",
    label: "MOVE FORWARD 0.5 M",
    instruction: "Slide forward to +0.50 m; keep heading and level fixed.",
    remaining_s: 14.8,
  },
  pose_output_to_cube: false,
  odometry_rows: 242,
  distance_m: 4.81,
  return_error_m: 1.23,
  position_m: [1.1, -0.55, 0.08],
  yaw: {
    lio_delta_deg: 18.6,
    cube_delta_deg: 18.2,
    cube_rate_dps: 2.1,
    cube_maximum_rate_dps: 8.4,
    rate_limit_dps: 30,
  },
  path: [
    [0, 0, 0],
    [0.9, 0, 0.03],
    [1.2, -0.55, 0.07],
    [1.1, -0.55, 0.08],
  ],
  cube_messages: 1760,
  cube_local_position_rows: 740,
  cube_attitude_fresh: true,
  cube_attitude_age_ms: 18,
  cube_local_position_fresh: true,
  cube_local_position_age_ms: 24,
  translation: {
    cube_body_delta_m: [0.48, 0.02, 0.01],
    cube_horizontal_speed_mps: 0.08,
    cube_maximum_horizontal_speed_mps: 0.12,
    speed_limit_mps: 0.5,
    cube_maximum_yaw_deviation_deg: 1.2,
    yaw_deviation_limit_deg: 10,
  },
  imu: {
    connected: true,
    rate_hz: 200.3,
    queue_drops: 0,
    clock_ready: true,
    clock_p95_ms: 3.8,
  },
  lidar: {
    connected: true,
    rate_hz: 5,
    queue_drops: 0,
    clock_ready: true,
    clock_p95_ms: 1.1,
  },
};

const browser = await chromium.launch({ headless: true });
const viewports = [
  { name: "desktop", width: 1440, height: 900 },
  { name: "mobile", width: 390, height: 844 },
];

try {
  for (const viewport of viewports) {
    const page = await browser.newPage({ viewport });
    const errors = [];
    page.on("console", (message) => {
      if (message.type() === "error") errors.push(message.text());
    });
    page.on("pageerror", (error) => errors.push(error.message));
    await page.route("**/api/lio-assist", async (route) => {
      await route.fulfill({
        contentType: "application/json",
        body: JSON.stringify(state),
      });
    });
    await page.goto(baseUrl, { waitUntil: "domcontentloaded" });
    await page.waitForFunction(
      () =>
        document.querySelector("#phaseLabel")?.textContent ===
        "MOVE FORWARD 0.5 M",
    );

    const result = await page.evaluate(() => {
      const canvas = document.querySelector("#pathCanvas");
      const context = canvas.getContext("2d");
      const pixels = context.getImageData(
        0,
        0,
        canvas.width,
        canvas.height,
      ).data;
      let changedPixels = 0;
      for (let index = 0; index < pixels.length; index += 4) {
        const difference =
          Math.abs(pixels[index] - 17) +
          Math.abs(pixels[index + 1] - 21) +
          Math.abs(pixels[index + 2] - 22);
        if (difference > 25) changedPixels += 1;
      }
      const phase = document
        .querySelector(".phase-band")
        .getBoundingClientRect();
      const workspace = document
        .querySelector(".workspace")
        .getBoundingClientRect();
      return {
        changedPixels,
        phaseLabel: document.querySelector("#phaseLabel")?.textContent,
        assistTitle: document.querySelector("#assistTitle")?.textContent,
        timelineLabels: document.querySelectorAll(".timeline-labels span").length,
        distance: document.querySelector("#distanceValue")?.textContent,
        position: document.querySelector("#positionReadout")?.textContent,
        phaseBottom: phase.bottom,
        workspaceTop: workspace.top,
        bodyWidth: document.body.scrollWidth,
        viewportWidth: window.innerWidth,
      };
    });
    if (errors.length) {
      throw new Error(`${viewport.name}: ${errors.join(" | ")}`);
    }
    if (result.changedPixels < 500) {
      throw new Error(`${viewport.name}: trajectory canvas appears blank`);
    }
    if (
      result.phaseLabel !== "MOVE FORWARD 0.5 M" ||
      result.assistTitle !== "Translation scale validation" ||
      result.timelineLabels !== 6 ||
      result.distance !== "+0.48 m" ||
      !result.position.includes("LIO X +1.10")
    ) {
      throw new Error(
        `${viewport.name}: telemetry did not render: ${JSON.stringify(result)}`,
      );
    }
    if (
      result.workspaceTop < result.phaseBottom ||
      result.bodyWidth > result.viewportWidth + 1
    ) {
      throw new Error(
        `${viewport.name}: layout overlaps or overflows: ${JSON.stringify(result)}`,
      );
    }
    await page.screenshot({
      path: path.join(outputDir, `${viewport.name}-lio-assist.png`),
      fullPage: true,
    });
    await page.close();
  }
} finally {
  await browser.close();
}
