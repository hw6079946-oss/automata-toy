import * as THREE from 'three';

export interface ProceduralTextureSet {
  beechMap: THREE.CanvasTexture;
  walnutMap: THREE.CanvasTexture;
  woodBumpMap: THREE.CanvasTexture;
  brushedMetalRoughness: THREE.CanvasTexture;
  workbenchMatMap: THREE.CanvasTexture;
  contactShadowMap: THREE.CanvasTexture;
  dispose: () => void;
}

function makeWoodCanvas(
  baseHex: string,
  grainHex: string,
  highlightHex: string,
  seed: number
): HTMLCanvasElement {
  const canvas = document.createElement('canvas');
  canvas.width = 512;
  canvas.height = 512;
  const ctx = canvas.getContext('2d')!;

  ctx.fillStyle = baseHex;
  ctx.fillRect(0, 0, 512, 512);

  // Subtle plank band variation
  for (let y = 0; y < 512; y += 2) {
    const wave = Math.sin(y * 0.045 + seed) * 0.5 + Math.cos(y * 0.11 + seed * 2) * 0.5;
    ctx.fillStyle = wave > 0 ? highlightHex : grainHex;
    ctx.globalAlpha = 0.05 + Math.abs(wave) * 0.06;
    ctx.fillRect(0, y, 512, 2);
  }

  // Fine organic wood fibers
  ctx.globalAlpha = 0.14;
  ctx.strokeStyle = grainHex;
  for (let i = 0; i < 95; i++) {
    const yBase = ((i * 37 + seed * 13) % 512);
    ctx.lineWidth = 0.8 + ((i * 7) % 3) * 0.45;
    ctx.beginPath();
    ctx.moveTo(0, yBase);
    for (let x = 0; x <= 512; x += 32) {
      const offset = Math.sin(x * 0.02 + i * 0.7 + seed) * 5.5;
      ctx.lineTo(x, yBase + offset);
    }
    ctx.stroke();
  }
  ctx.globalAlpha = 1;
  return canvas;
}

function makeWoodBumpCanvas(): HTMLCanvasElement {
  const canvas = document.createElement('canvas');
  canvas.width = 256;
  canvas.height = 256;
  const ctx = canvas.getContext('2d')!;
  ctx.fillStyle = '#808080';
  ctx.fillRect(0, 0, 256, 256);
  for (let i = 0; i < 80; i++) {
    const y = (i * 19) % 256;
    ctx.strokeStyle = i % 2 === 0 ? '#9c9c9c' : '#646464';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(0, y);
    for (let x = 0; x <= 256; x += 16) {
      ctx.lineTo(x, y + Math.sin(x * 0.04 + i) * 3);
    }
    ctx.stroke();
  }
  return canvas;
}

function makeBrushedMetalCanvas(): HTMLCanvasElement {
  const canvas = document.createElement('canvas');
  canvas.width = 256;
  canvas.height = 256;
  const ctx = canvas.getContext('2d')!;
  ctx.fillStyle = '#909090';
  ctx.fillRect(0, 0, 256, 256);
  for (let y = 0; y < 256; y++) {
    const v = 120 + Math.floor(Math.sin(y * 1.7) * 18 + ((y * 37) % 24));
    ctx.fillStyle = `rgb(${v},${v},${v})`;
    ctx.fillRect(0, y, 256, 1);
  }
  return canvas;
}

function makeWorkbenchMatCanvas(): HTMLCanvasElement {
  const canvas = document.createElement('canvas');
  canvas.width = 512;
  canvas.height = 512;
  const ctx = canvas.getContext('2d')!;

  ctx.fillStyle = '#E6E1D6';
  ctx.fillRect(0, 0, 512, 512);

  // Subtle architectural drafting grid
  ctx.strokeStyle = 'rgba(28, 25, 23, 0.055)';
  ctx.lineWidth = 1;
  const step = 32;
  for (let i = 0; i <= 512; i += step) {
    ctx.beginPath();
    ctx.moveTo(i, 0);
    ctx.lineTo(i, 512);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(0, i);
    ctx.lineTo(512, i);
    ctx.stroke();
  }

  // Major 128px grid lines
  ctx.strokeStyle = 'rgba(28, 25, 23, 0.11)';
  ctx.lineWidth = 1.5;
  for (let i = 0; i <= 512; i += 128) {
    ctx.beginPath();
    ctx.moveTo(i, 0);
    ctx.lineTo(i, 512);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(0, i);
    ctx.lineTo(512, i);
    ctx.stroke();
  }
  return canvas;
}

function makeContactShadowCanvas(): HTMLCanvasElement {
  const canvas = document.createElement('canvas');
  canvas.width = 256;
  canvas.height = 256;
  const ctx = canvas.getContext('2d')!;
  const grad = ctx.createRadialGradient(128, 128, 24, 128, 128, 124);
  grad.addColorStop(0, 'rgba(24, 20, 16, 0.55)');
  grad.addColorStop(0.55, 'rgba(24, 20, 16, 0.24)');
  grad.addColorStop(1, 'rgba(24, 20, 16, 0.0)');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, 256, 256);
  return canvas;
}

export function createProceduralTextures(maxAnisotropy: number): ProceduralTextureSet {
  const beechMap = new THREE.CanvasTexture(makeWoodCanvas('#D8B589', '#A87C4F', '#E7CCA5', 1.3));
  beechMap.colorSpace = THREE.SRGBColorSpace;
  beechMap.wrapS = THREE.RepeatWrapping;
  beechMap.wrapT = THREE.RepeatWrapping;
  beechMap.anisotropy = maxAnisotropy;

  const walnutMap = new THREE.CanvasTexture(makeWoodCanvas('#6C4932', '#442B1B', '#855E42', 4.7));
  walnutMap.colorSpace = THREE.SRGBColorSpace;
  walnutMap.wrapS = THREE.RepeatWrapping;
  walnutMap.wrapT = THREE.RepeatWrapping;
  walnutMap.anisotropy = maxAnisotropy;

  const woodBumpMap = new THREE.CanvasTexture(makeWoodBumpCanvas());
  woodBumpMap.wrapS = THREE.RepeatWrapping;
  woodBumpMap.wrapT = THREE.RepeatWrapping;

  const brushedMetalRoughness = new THREE.CanvasTexture(makeBrushedMetalCanvas());
  brushedMetalRoughness.wrapS = THREE.RepeatWrapping;
  brushedMetalRoughness.wrapT = THREE.RepeatWrapping;

  const workbenchMatMap = new THREE.CanvasTexture(makeWorkbenchMatCanvas());
  workbenchMatMap.colorSpace = THREE.SRGBColorSpace;
  workbenchMatMap.wrapS = THREE.RepeatWrapping;
  workbenchMatMap.wrapT = THREE.RepeatWrapping;
  workbenchMatMap.repeat.set(8, 8);
  workbenchMatMap.anisotropy = maxAnisotropy;

  const contactShadowMap = new THREE.CanvasTexture(makeContactShadowCanvas());

  return {
    beechMap,
    walnutMap,
    woodBumpMap,
    brushedMetalRoughness,
    workbenchMatMap,
    contactShadowMap,
    dispose: () => {
      beechMap.dispose();
      walnutMap.dispose();
      woodBumpMap.dispose();
      brushedMetalRoughness.dispose();
      workbenchMatMap.dispose();
      contactShadowMap.dispose();
    },
  };
}
