import { resolve } from "node:path";
import { defineConfig } from "vite";

export default defineConfig({
  build: {
    rollupOptions: {
      input: {
        main: resolve(import.meta.dirname, "index.html"),
        lioAssist: resolve(import.meta.dirname, "lio-assist.html"),
        slamPoc: resolve(import.meta.dirname, "slam-poc.html"),
      },
    },
  },
});
