import { useCallback, useEffect, useRef, useState } from 'react';
import { BottomControlDock, type DockTabId } from './components/BottomControlDock';
import { MechanismAtlasModal } from './components/MechanismAtlasModal';
import { TopBar } from './components/TopBar';
import { clampModuleSlotX, findNearestFreeSlotX } from './engine/kinematics';
import {
  CLASSIC_PRESETS,
  createDefaultModule,
  EMPTY_FRAME_BLUEPRINT,
} from './engine/presets';
import type {
  AutomataBlueprint,
  AutomataModule,
  CameraPresetId,
  MaterialFinishId,
  ModuleType,
} from './engine/types';
import { AutomataViewport } from './scene/AutomataViewport';

export default function App() {
  const [blueprint, setBlueprint] = useState<AutomataBlueprint>(() => ({
    ...CLASSIC_PRESETS[0],
    modules: CLASSIC_PRESETS[0].modules.map((m) => ({ ...m })),
  }));
  const [selectedModuleId, setSelectedModuleId] = useState<string | null>(
    () => CLASSIC_PRESETS[0].modules[0]?.id ?? null
  );
  const [isPlaying, setIsPlaying] = useState(true);
  const [rpm, setRpm] = useState(22);
  const [manualAngleDeg, setManualAngleDeg] = useState(0);
  const [cutawayAmount, setCutawayAmount] = useState(0.65);
  const [explodeAmount, setExplodeAmount] = useState(0);
  const [cameraPreset, setCameraPreset] = useState<CameraPresetId>('three-quarter');
  const [cameraPresetTrigger, setCameraPresetTrigger] = useState(0);

  const [dockCollapsed, setDockCollapsed] = useState(false);
  const [activeTab, setActiveTab] = useState<DockTabId>('build');
  const [atlasOpen, setAtlasOpen] = useState(false);
  const [toastMsg, setToastMsg] = useState<string | null>(null);

  const moduleSeedRef = useRef(10);
  const toastTimerRef = useRef<number | null>(null);

  const showToast = useCallback((msg: string) => {
    setToastMsg(msg);
    if (toastTimerRef.current) {
      window.clearTimeout(toastTimerRef.current);
    }
    toastTimerRef.current = window.setTimeout(() => {
      setToastMsg(null);
    }, 2600);
  }, []);

  // Keyboard shortcuts: H toggles dock, Space toggles play/pause
  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      const target = e.target as HTMLElement | null;
      if (
        target &&
        (target.tagName === 'INPUT' ||
          target.tagName === 'SELECT' ||
          target.tagName === 'TEXTAREA')
      ) {
        return;
      }
      if (e.key === 'h' || e.key === 'H') {
        setDockCollapsed((prev) => !prev);
      } else if (e.code === 'Space' && !atlasOpen) {
        e.preventDefault();
        setIsPlaying((prev) => !prev);
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [atlasOpen]);

  const handleLoadPreset = useCallback(
    (presetId: string) => {
      const found = CLASSIC_PRESETS.find((p) => p.id === presetId);
      if (!found) return;
      const cloned: AutomataBlueprint = {
        ...found,
        modules: found.modules.map((m) => ({ ...m })),
      };
      setBlueprint(cloned);
      setSelectedModuleId(cloned.modules[0]?.id ?? null);
      showToast(`已加载经典样板：${cloned.name}`);
    },
    [showToast]
  );

  const handleClearFrame = useCallback(() => {
    setBlueprint({
      ...EMPTY_FRAME_BLUEPRINT,
      finish: blueprint.finish,
      modules: [],
    });
    setSelectedModuleId(null);
    setActiveTab('build');
    setDockCollapsed(false);
    showToast('已重置为空白木制底座，请拖拽积木开始搭建');
  }, [blueprint.finish, showToast]);

  const handleAddModule = useCallback(
    (type: ModuleType, preferredX?: number) => {
      setBlueprint((prev) => {
        if (prev.modules.length >= 6) {
          showToast('主轴最多同时挂载 6 组大型积木模块');
          return prev;
        }
        const slotX = findNearestFreeSlotX(preferredX ?? 0, prev.modules);
        const seed = ++moduleSeedRef.current;
        const newMod = createDefaultModule(type, slotX, seed);
        setSelectedModuleId(newMod.id);
        setActiveTab('build');
        showToast(`已装配：${newMod.label}`);
        return {
          ...prev,
          id: 'custom-work',
          name: prev.modules.length === 0 ? '自定义木偶机构设计' : prev.name,
          modules: [...prev.modules, newMod],
        };
      });
    },
    [showToast]
  );

  const handleUpdateModule = useCallback((id: string, patch: Partial<AutomataModule>) => {
    setBlueprint((prev) => ({
      ...prev,
      modules: prev.modules.map((m) => {
        if (m.id !== id) return m;
        const nextSlotX =
          patch.slotX !== undefined
            ? clampModuleSlotX(patch.slotX, id, prev.modules)
            : m.slotX;
        return {
          ...m,
          ...patch,
          slotX: nextSlotX,
        } as AutomataModule;
      }),
    }));
  }, []);

  const handleMoveModuleSlotX = useCallback((id: string, slotX: number) => {
    setBlueprint((prev) => ({
      ...prev,
      modules: prev.modules.map((m) => (m.id === id ? { ...m, slotX } : m)),
    }));
  }, []);

  const handleMoveFourBarPivot = useCallback((id: string, pivotZ: number, pivotY: number) => {
    setBlueprint((prev) => ({
      ...prev,
      modules: prev.modules.map((m) =>
        m.id === id && m.type === 'four-bar-linkage'
          ? { ...m, pivotZ, pivotY }
          : m
      ),
    }));
  }, []);

  const handleMoveCircularCamOffset = useCallback((id: string, camOffsetRatio: number) => {
    setBlueprint((prev) => ({
      ...prev,
      modules: prev.modules.map((m) =>
        m.id === id && (m.type === 'circular-cam' || m.type === 'semi-circular-cam')
          ? { ...m, camOffsetRatio }
          : m
      ),
    }));
  }, []);

  const handleDeleteModule = useCallback(
    (id: string) => {
      setBlueprint((prev) => {
        const remaining = prev.modules.filter((m) => m.id !== id);
        if (selectedModuleId === id) {
          setSelectedModuleId(remaining[0]?.id ?? null);
        }
        return { ...prev, modules: remaining };
      });
      showToast('已移除机械积木组件');
    },
    [selectedModuleId, showToast]
  );

  const handleChangeFinish = useCallback((finish: MaterialFinishId) => {
    setBlueprint((prev) => ({ ...prev, finish }));
  }, []);

  const handleChangeManualAngle = useCallback((deg: number) => {
    setIsPlaying(false);
    setManualAngleDeg(deg);
  }, []);

  const handleSelectCameraPreset = useCallback((preset: CameraPresetId) => {
    setCameraPreset(preset);
    setCameraPresetTrigger((c) => c + 1);
  }, []);

  const handleExportBlueprint = useCallback(() => {
    const json = JSON.stringify(blueprint, null, 2);
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${blueprint.id || 'automaton-blueprint'}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    showToast('已导出当前 Automata 机械蓝图 JSON');
  }, [blueprint, showToast]);

  return (
    <div className="w-screen h-screen flex flex-col overflow-hidden bg-[#EDE9E1] text-[#1C1917]">
      <TopBar
        isPlaying={isPlaying}
        onTogglePlay={() => setIsPlaying((p) => !p)}
        onOpenAtlas={() => setAtlasOpen(true)}
        onSelectPresetsTab={() => {
          setDockCollapsed(false);
          setActiveTab('presets');
        }}
        onClearFrame={handleClearFrame}
        onExportBlueprint={handleExportBlueprint}
      />

      <main
        className="relative flex-1 min-h-0 w-full"
        data-state={isPlaying ? 'playing' : 'paused'}
        data-module-count={blueprint.modules.length}
      >
        <AutomataViewport
          blueprint={blueprint}
          selectedModuleId={selectedModuleId}
          isPlaying={isPlaying}
          rpm={rpm}
          manualAngleDeg={manualAngleDeg}
          cutawayAmount={cutawayAmount}
          explodeAmount={explodeAmount}
          cameraPreset={cameraPreset}
          cameraPresetTrigger={cameraPresetTrigger}
          onSelectModule={(id) => {
            setSelectedModuleId(id);
            if (id) setActiveTab('build');
          }}
          onMoveModuleSlotX={handleMoveModuleSlotX}
          onMoveFourBarPivot={handleMoveFourBarPivot}
          onMoveCircularCamOffset={handleMoveCircularCamOffset}
          onDropNewModule={(type, slotX) => handleAddModule(type, slotX)}
          onAngleSync={setManualAngleDeg}
        />

        {/* Minimal Editorial Specimen Title Strip (Top-Left, non-blocking, zero pills) */}
        <div className="pointer-events-none absolute top-4 left-6 z-10 max-w-md space-y-0.5">
          <h1 className="font-display text-xl sm:text-2xl font-semibold text-[#1C1917] leading-tight">
            {blueprint.name}
          </h1>
          <div className="flex flex-wrap items-center gap-2 text-xs text-[#57534E]">
            <span>{blueprint.styleCredit}</span>
            <span aria-hidden="true">·</span>
            <span>拖拽组件可整体移动，拖动红环可单独调连杆/圆形凸轮</span>
          </div>
        </div>

        {/* Non-blocking Toast */}
        {toastMsg && (
          <div
            role="status"
            aria-live="polite"
            className="pointer-events-none absolute top-4 right-6 z-20 px-3.5 py-2 rounded-lg bg-[#1C1917] text-[#FAF8F4] text-xs font-medium shadow-md"
          >
            {toastMsg}
          </div>
        )}

        <BottomControlDock
          collapsed={dockCollapsed}
          onToggleCollapse={() => setDockCollapsed((c) => !c)}
          activeTab={activeTab}
          onChangeTab={setActiveTab}
          blueprint={blueprint}
          selectedModuleId={selectedModuleId}
          onSelectModule={setSelectedModuleId}
          onAddModule={(type) => handleAddModule(type)}
          onUpdateModule={handleUpdateModule}
          onDeleteModule={handleDeleteModule}
          isPlaying={isPlaying}
          rpm={rpm}
          manualAngleDeg={manualAngleDeg}
          cutawayAmount={cutawayAmount}
          explodeAmount={explodeAmount}
          cameraPreset={cameraPreset}
          onTogglePlay={() => setIsPlaying((p) => !p)}
          onChangeRpm={setRpm}
          onChangeManualAngle={handleChangeManualAngle}
          onChangeCutaway={setCutawayAmount}
          onChangeExplode={setExplodeAmount}
          onSelectCameraPreset={handleSelectCameraPreset}
          onChangeFinish={handleChangeFinish}
          onLoadPreset={handleLoadPreset}
          onClearFrame={handleClearFrame}
        />
      </main>

      <MechanismAtlasModal
        open={atlasOpen}
        onClose={() => setAtlasOpen(false)}
        onLoadPreset={handleLoadPreset}
      />
    </div>
  );
}
