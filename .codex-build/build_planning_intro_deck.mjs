import fs from "node:fs/promises";
import path from "node:path";
import { pathToFileURL } from "node:url";
import { Presentation, PresentationFile } from "@oai/artifact-tool";

const SKILL_DIR = "/Users/sangbum/.codex/plugins/cache/openai-primary-runtime/presentations/26.909.12148/skills/presentations";
const workspaceDir = "/Users/sangbum/Workspace/IFAC_ws";
const buildDir = path.join(workspaceDir, ".codex-build", "planning-intro");
const stagingDir = path.join(workspaceDir, ".codex-finalizer", "planning-intro");
const finalPath = path.join(workspaceDir, "presentation_output", "planning_intro_2slides_MIRU_v2.pptx");
const referencePath = "/Users/sangbum/Workspace/planning_ws_MIRU_large_text.pptx";

await fs.mkdir(buildDir, { recursive: true });
await fs.mkdir(stagingDir, { recursive: true });
await fs.mkdir(path.dirname(finalPath), { recursive: true });

const NAVY = "#002060";
const MUTED = "#647089";
const WHITE = "#FFFFFF";
const FONT = "Arial";

const miruLogo = await fs.readFile(path.join(workspaceDir, ".codex-build", "miru-assets", "image1.png"));
const hanyangLogo = await fs.readFile(path.join(workspaceDir, ".codex-build", "miru-assets", "image2.png"));
const architecture = await fs.readFile(path.join(buildDir, "architecture_content_v3.png"));

const presentation = Presentation.create({ slideSize: { width: 1280, height: 720 } });
presentation.theme.colorScheme = {
  name: "MIRU",
  themeColors: {
    accent1: NAVY,
    accent2: "#0E948B",
    accent3: "#ED8A23",
    accent4: "#3263AB",
    accent5: "#7E56A6",
    accent6: "#D84640",
    bg1: WHITE,
    bg2: "#F8FAFD",
    tx1: "#1C2534",
    tx2: MUTED,
    dk1: "#000000",
    dk2: NAVY,
    lt1: WHITE,
    lt2: "#DCE5F2",
    hlink: "#3263AB",
    folHlink: "#7E56A6",
  },
};

function addRect(slide, position, fill) {
  return slide.shapes.add({
    geometry: "rect",
    position,
    fill,
    line: { fill: "none", width: 0 },
  });
}

function addText(slide, value, position, options = {}) {
  const shape = slide.shapes.add({
    geometry: "textbox",
    position,
    fill: "none",
    line: { fill: "none", width: 0 },
  });
  shape.text = value;
  shape.text.style = {
    typeface: FONT,
    fontSize: options.fontSize ?? 28,
    bold: options.bold ?? false,
    color: options.color ?? NAVY,
    alignment: options.alignment ?? "left",
    verticalAlignment: options.verticalAlignment ?? "middle",
    autoFit: "shrinkText",
    wrap: "square",
    insets: { top: 0, right: 0, bottom: 0, left: 0 },
  };
  return shape;
}

function addTemplateMarks(slide, withCorner = true) {
  if (withCorner) {
    const corner = slide.shapes.add({
      geometry: "rtTriangle",
      position: { left: 0, top: 0, width: 27, height: 27 },
      fill: NAVY,
      line: { fill: "none", width: 0 },
    });
    corner.rotation = 90;
  }
  slide.images.add({
    blob: hanyangLogo,
    contentType: "image/png",
    alt: "Hanyang University logo",
    fit: "contain",
    position: { left: 13, top: 672, width: 205, height: 38 },
  });
  slide.images.add({
    blob: miruLogo,
    contentType: "image/png",
    alt: "MIRU logo",
    fit: "contain",
    position: { left: 1190, top: 655, width: 78, height: 58 },
  });
}

// Slide 1: minimal section cover in the MIRU template rhythm.
{
  const slide = presentation.slides.add();
  slide.background.fill = WHITE;
  addTemplateMarks(slide, false);
  addRect(slide, { left: 160, top: 232, width: 960, height: 10 }, NAVY);
  addText(slide, "Planning", { left: 160, top: 280, width: 960, height: 90 }, {
    fontSize: 68,
    bold: true,
    alignment: "center",
  });
  addText(slide, "Global & Local Planning", { left: 160, top: 378, width: 960, height: 42 }, {
    fontSize: 25,
    color: MUTED,
    alignment: "center",
  });
  addRect(slide, { left: 160, top: 447, width: 960, height: 1 }, NAVY);
}

// Slide 2: overview image, cropped to remove the image's own header.
{
  const slide = presentation.slides.add();
  slide.background.fill = WHITE;
  addTemplateMarks(slide, true);
  addText(slide, "Planning Architecture", { left: 68, top: 28, width: 720, height: 48 }, {
    fontSize: 35,
    bold: true,
  });
  addText(slide, "기본 레이스라인과 장애물 정보를 Local Waypoints로 연결", { left: 70, top: 75, width: 850, height: 26 }, {
    fontSize: 18,
    color: MUTED,
  });
  addRect(slide, { left: 58, top: 105, width: 1164, height: 2 }, NAVY);
  slide.images.add({
    blob: architecture,
    contentType: "image/png",
    alt: "Planning 입력, 장애물 회피, Local 경로 생성과 Local Waypoints의 전체 구조",
    fit: "contain",
    position: { left: 65, top: 116, width: 1150, height: 535 },
  });
}

const candidatePath = path.join(stagingDir, "candidate.pptx");
await (await PresentationFile.exportPptx(presentation)).save(candidatePath);

for (let i = 0; i < presentation.slides.items.length; i += 1) {
  const slide = presentation.slides.getItem(i);
  const preview = await presentation.export({ slide, format: "png", scale: 1.5 });
  await fs.writeFile(path.join(buildDir, `slide-${i + 1}.png`), new Uint8Array(await preview.arrayBuffer()));
  const layout = await slide.export({ format: "layout" });
  await fs.writeFile(path.join(buildDir, `slide-${i + 1}.layout.json`), await layout.text());
}

const { finalizePresentation } = await import(pathToFileURL(
  path.join(SKILL_DIR, "container_tools", "artifact_tool_utils.mjs"),
).href);

await finalizePresentation({
  explicitTotalSlideCount: 2,
  requiredNativeTableOwnerSlides: [],
  requiredNativeChartOwnerSlides: [],
  workspaceDir,
  candidatePath,
  finalPath,
  pythonExecutable: "/Users/sangbum/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3",
  integrityValidatorPath: path.join(SKILL_DIR, "container_tools", "inspect_presentation_package_integrity.py"),
  layoutValidatorPath: path.join(SKILL_DIR, "container_tools", "inspect_presentation_layout_geometry.py"),
  layoutArgs: [
    "--expected-slide-size-emu", "12192000,6858000",
    "--validate-heading-fit",
  ],
  fontPolicy: {
    basis: "reference",
    families: ["Arial"],
    scriptFonts: { ea: "Arial" },
    referencePath,
    referenceSha256: "86b4ead199ce4d5f59c83ee09417ccc8c12d2ce1cda01fc6bc3f1048b4f34ed2",
  },
  verifyArtifactToolImport: true,
  receiptPath: path.join(stagingDir, "planning_intro_2slides_MIRU_v2.validation.json"),
});

console.log(finalPath);
