import * as THREE from 'three';
import type { MaterialFinishId } from '../engine/types';
import type { ProceduralTextureSet } from './textures';

export interface AutomataMaterialSet {
  woodPrimary: THREE.MeshStandardMaterial;
  woodSecondary: THREE.MeshStandardMaterial;
  woodDarkTrim: THREE.MeshStandardMaterial;
  metalHardware: THREE.MeshPhysicalMaterial;
  steelShaft: THREE.MeshStandardMaterial;
  paintedAccent: THREE.MeshStandardMaterial;
  paintedSecondary: THREE.MeshStandardMaterial;
  paintedIvory: THREE.MeshStandardMaterial;
  cutawayGlass: THREE.MeshPhysicalMaterial;
  selectionHighlight: THREE.MeshStandardMaterial;
  pegHoleDark: THREE.MeshStandardMaterial;
  applyFinish: (finish: MaterialFinishId) => void;
  setCutawayOpacity: (cutawayAmount: number) => void;
  dispose: () => void;
}

export function createAutomataMaterials(
  textures: ProceduralTextureSet,
  isSoftwareGl: boolean
): AutomataMaterialSet {
  const woodPrimary = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    map: textures.beechMap,
    bumpMap: textures.woodBumpMap,
    bumpScale: 0.012,
    roughness: 0.68,
    metalness: 0.02,
  });

  const woodSecondary = new THREE.MeshStandardMaterial({
    color: 0xf4e4ce,
    map: textures.beechMap,
    bumpMap: textures.woodBumpMap,
    bumpScale: 0.015,
    roughness: 0.74,
    metalness: 0.02,
  });

  const woodDarkTrim = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    map: textures.walnutMap,
    bumpMap: textures.woodBumpMap,
    bumpScale: 0.012,
    roughness: 0.62,
    metalness: 0.04,
  });

  const metalHardware = new THREE.MeshPhysicalMaterial({
    color: 0xd6a648,
    roughness: 0.24,
    metalness: 0.92,
    roughnessMap: textures.brushedMetalRoughness,
    clearcoat: 0.2,
    anisotropy: isSoftwareGl ? 0 : 0.65,
  });

  const steelShaft = new THREE.MeshStandardMaterial({
    color: 0x9aa3ad,
    roughness: 0.28,
    metalness: 0.88,
    roughnessMap: textures.brushedMetalRoughness,
  });

  const paintedAccent = new THREE.MeshStandardMaterial({
    color: 0xc2410c,
    roughness: 0.42,
    metalness: 0.05,
  });

  const paintedSecondary = new THREE.MeshStandardMaterial({
    color: 0x1d4e68,
    roughness: 0.46,
    metalness: 0.05,
  });

  const paintedIvory = new THREE.MeshStandardMaterial({
    color: 0xf7f2e7,
    roughness: 0.55,
    metalness: 0.02,
  });

  const cutawayGlass = new THREE.MeshPhysicalMaterial({
    color: 0xe6eff2,
    roughness: 0.08,
    metalness: 0.0,
    transparent: true,
    opacity: isSoftwareGl ? 0.22 : 0.24,
    depthWrite: false,
    ior: 1.48,
    clearcoat: 0.9,
  });

  const selectionHighlight = new THREE.MeshStandardMaterial({
    color: 0xea580c,
    emissive: 0xb45309,
    emissiveIntensity: 0.45,
    roughness: 0.3,
    metalness: 0.4,
  });

  const pegHoleDark = new THREE.MeshStandardMaterial({
    color: 0x29231f,
    roughness: 0.88,
    metalness: 0.05,
  });

  function applyFinish(finish: MaterialFinishId) {
    if (finish === 'beech-brass') {
      woodPrimary.map = textures.beechMap;
      woodPrimary.color.setHex(0xffffff);
      woodSecondary.map = textures.beechMap;
      woodSecondary.color.setHex(0xf1dfc8);
      woodDarkTrim.map = textures.walnutMap;
      woodDarkTrim.color.setHex(0xffffff);
      metalHardware.color.setHex(0xd6a648);
      paintedAccent.color.setHex(0xc2410c);
      paintedSecondary.color.setHex(0x1d4e68);
    } else if (finish === 'walnut-copper') {
      woodPrimary.map = textures.walnutMap;
      woodPrimary.color.setHex(0xffffff);
      woodSecondary.map = textures.beechMap;
      woodSecondary.color.setHex(0xdfc39d);
      woodDarkTrim.map = textures.walnutMap;
      woodDarkTrim.color.setHex(0x887266);
      metalHardware.color.setHex(0xc8744b);
      paintedAccent.color.setHex(0x155e75);
      paintedSecondary.color.setHex(0x9a3412);
    } else {
      // folk-painted
      woodPrimary.map = textures.beechMap;
      woodPrimary.color.setHex(0x4a6b5d);
      woodSecondary.map = textures.beechMap;
      woodSecondary.color.setHex(0xf6ead7);
      woodDarkTrim.map = textures.walnutMap;
      woodDarkTrim.color.setHex(0x8c3a2b);
      metalHardware.color.setHex(0xdfb15b);
      paintedAccent.color.setHex(0xd93829);
      paintedSecondary.color.setHex(0x28587b);
    }
    woodPrimary.needsUpdate = true;
    woodSecondary.needsUpdate = true;
    woodDarkTrim.needsUpdate = true;
  }

  function setCutawayOpacity(cutawayAmount: number) {
    // cutawayAmount 0 = full front window visible, 1 = fully hidden X-ray inspection
    const alpha = THREE.MathUtils.clamp(1 - cutawayAmount, 0, 1);
    cutawayGlass.opacity = alpha * 0.28;
    cutawayGlass.visible = alpha > 0.03;
  }

  return {
    woodPrimary,
    woodSecondary,
    woodDarkTrim,
    metalHardware,
    steelShaft,
    paintedAccent,
    paintedSecondary,
    paintedIvory,
    cutawayGlass,
    selectionHighlight,
    pegHoleDark,
    applyFinish,
    setCutawayOpacity,
    dispose: () => {
      woodPrimary.dispose();
      woodSecondary.dispose();
      woodDarkTrim.dispose();
      metalHardware.dispose();
      steelShaft.dispose();
      paintedAccent.dispose();
      paintedSecondary.dispose();
      paintedIvory.dispose();
      cutawayGlass.dispose();
      selectionHighlight.dispose();
      pegHoleDark.dispose();
    },
  };
}
