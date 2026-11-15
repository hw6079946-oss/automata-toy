import React, { useMemo } from 'react';
import {
  ChevronDown,
  ChevronUp,
  CircleDot,
  Cog,
  Contrast,
  Disc,
  Disc2,
  GitCommitHorizontal,
  Plus,
  Repeat,
  StepForward,
  Trash2,
} from 'lucide-react';
import { sampleCycleCurve } from '../engine/kinematics';
import { CLASSIC_PRESETS } from '../engine/presets';
import type {
  AutomataBlueprint,
  AutomataModule,
  CameraPresetId,
  CamProfileType,
  MaterialFinishId,
  ModuleType,
  TopperType,
} from '../engine/types';

export type DockTabId = 'build' | 'motion' | 'presets';

interface BottomControlDockProps {
  collapsed: boolean;
  onToggleCollapse: () => void;
  activeTab: DockTabId;
  onChangeTab: (tab: DockTabId) => void;
  blueprint: AutomataBlueprint;
  selectedModuleId: string | null;
  onSelectModule: (id: string | null) => void;
  onAddModule: (type: ModuleType) => void;
  onUpdateModule: (id: string, patch: Partial<AutomataModule>) => void;
  onDeleteModule: (id: string) => void;
  isPlaying: boolean;
  rpm: number;
  manualAngleDeg: number;
  cutawayAmount: number;
  explodeAmount: number;
  cameraPreset: CameraPresetId;
  onTogglePlay: () => void;
  onChangeRpm: (rpm: number) => void;
  onChangeManualAngle: (deg: number) => void;
  onChangeCutaway: (v: number) => void;
  onChangeExplode: (v: number) => void;
  onSelectCameraPreset: (preset: CameraPresetId) => void;
  onChangeFinish: (finish: MaterialFinishId) => void;
  onLoadPreset: (presetId: string) => void;
  onClearFrame: () => void;
}

const CAM_LABELS: Array<{ id: CamProfileType; name: string }> = [
  { id: 'pear', name: '蛋形凸轮' },
  { id: 'eccentric', name: '偏心圆轮' },
  { id: 'snail', name: '蜗牛骤降' },
  { id: 'heart', name: '心形双峰' },
];

const TOPPER_LABELS: Array<{ id: TopperType; name: string }> = [
  { id: 'seagull', name: '振翅海鸥' },
  { id: 'woodpecker', name: '啄木鸟' },
  { id: 'boatman', name: '划桨水手' },
  { id: 'windmill', name: '木构风车' },
  { id: 'propeller', name: '十字螺旋桨' },
  { id: 'arrow', name: '指示箭头' },
  { id: 'none', name: '裸露顶杆' },
];

const FINISH_OPTIONS: Array<{ id: MaterialFinishId; name: string; woodHex: string; metalHex: string }> = [
  { id: 'beech-brass', name: '山毛榉 · 黄铜', woodHex: '#D8B589', metalHex: '#D6A648' },
  { id: 'walnut-copper', name: '胡桃木 · 红铜', woodHex: '#6C4932', metalHex: '#C8744B' },
  { id: 'folk-painted', name: '传统彩绘木偶', woodHex: '#4A6B5D', metalHex: '#D93829' },
];

const CAMERA_OPTIONS: Array<{ id: CameraPresetId; name: string }> = [
  { id: 'three-quarter', name: '经典斜视' },
  { id: 'front', name: '正面机构' },
  { id: 'side', name: '侧面连杆' },
  { id: 'top', name: '舞台俯视' },
];

export const BottomControlDock: React.FC<BottomControlDockProps> = ({
  collapsed,
  onToggleCollapse,
  activeTab,
  onChangeTab,
  blueprint,
  selectedModuleId,
  onSelectModule,
  onAddModule,
  onUpdateModule,
  onDeleteModule,
  isPlaying,
  rpm,
  manualAngleDeg,
  cutawayAmount,
  explodeAmount,
  cameraPreset,
  onTogglePlay,
  onChangeRpm,
  onChangeManualAngle,
  onChangeCutaway,
  onChangeExplode,
  onSelectCameraPreset,
  onChangeFinish,
  onLoadPreset,
  onClearFrame,
}) => {
  const selectedModule = useMemo(
    () => blueprint.modules.find((m) => m.id === selectedModuleId) ?? null,
    [blueprint.modules, selectedModuleId]
  );

  const cyclePath = useMemo(() => {
    if (!selectedModule) return '';
    const pts = sampleCycleCurve(selectedModule, 48);
    return pts
      .map((v, i) => {
        const x = (i / (pts.length - 1)) * 120;
        const y = 32 - v * 26;
        return `${i === 0 ? 'M' : 'L'}${x.toFixed(1)},${y.toFixed(1)}`;
      })
      .join(' ');
  }, [selectedModule]);

  const handleDragStart = (e: React.DragEvent<HTMLButtonElement>, type: ModuleType) => {
    e.dataTransfer.setData('application/automata-module', type);
    e.dataTransfer.effectAllowed = 'copy';
  };

  return (
    <div
      className="fixed bottom-4 left-1/2 -translate-x-1/2 z-20 flex flex-col items-center transition-transform duration-200 ease-out"
      style={{ width: 'min(920px, 94vw)' }}
    >
      {/* Collapse / Expand Handle */}
      <button
        type="button"
        onClick={onToggleCollapse}
        aria-expanded={!collapsed}
        aria-label={collapsed ? '展开工坊控制栏' : '收起工坊控制栏'}
        className="mb-1.5 inline-flex items-center gap-1.5 px-3.5 py-1 text-xs font-medium text-[#1C1917] bg-[#FAF8F4] border border-[#1C1917]/15 rounded-t-lg rounded-b-sm shadow-sm hover:bg-[#F3EFE6] transition-colors cursor-pointer"
      >
        {collapsed ? <ChevronUp className="w-3.5 h-3.5" /> : <ChevronDown className="w-3.5 h-3.5" />}
        <span>{collapsed ? '展开积木与控制面板 (H)' : '收起面板 · 沉浸赏析 (H)'}</span>
      </button>

      {!collapsed && (
        <section
          aria-label="机械工坊控制台"
          className="w-full bg-[#FAF8F4] border border-[#1C1917]/15 rounded-xl shadow-lg overflow-hidden"
        >
          {/* Dock Navigation Header */}
          <div className="flex flex-wrap items-center justify-between gap-3 px-4 py-2.5 border-b border-[#1C1917]/10 bg-[#F3EFE6]">
            <div className="flex items-center gap-1 p-0.5 bg-[#E5E0D5] rounded-lg" role="tablist">
              <button
                type="button"
                role="tab"
                aria-selected={activeTab === 'build'}
                onClick={() => onChangeTab('build')}
                className={`px-3 py-1.5 text-xs font-semibold rounded-md transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                  activeTab === 'build'
                    ? 'bg-[#FAF8F4] text-[#1C1917] shadow-sm'
                    : 'text-[#57534E] hover:text-[#1C1917]'
                }`}
              >
                积木装配与参数
              </button>
              <button
                type="button"
                role="tab"
                aria-selected={activeTab === 'motion'}
                onClick={() => onChangeTab('motion')}
                className={`px-3 py-1.5 text-xs font-semibold rounded-md transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                  activeTab === 'motion'
                    ? 'bg-[#FAF8F4] text-[#1C1917] shadow-sm'
                    : 'text-[#57534E] hover:text-[#1C1917]'
                }`}
              >
                运转与剖视视角
              </button>
              <button
                type="button"
                role="tab"
                aria-selected={activeTab === 'presets'}
                onClick={() => onChangeTab('presets')}
                className={`px-3 py-1.5 text-xs font-semibold rounded-md transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                  activeTab === 'presets'
                    ? 'bg-[#FAF8F4] text-[#1C1917] shadow-sm'
                    : 'text-[#57534E] hover:text-[#1C1917]'
                }`}
              >
                经典木偶与涂装
              </button>
            </div>

            {/* Compact Live Telemetry Readout (tabular numerals, zero pill clutter) */}
            <div className="flex items-center gap-3 text-xs text-[#57534E] font-mono-tabular">
              <span>组件 {blueprint.modules.length}</span>
              <span aria-hidden="true">·</span>
              <span>曲柄 {manualAngleDeg}°</span>
              <span aria-hidden="true">·</span>
              <span>{isPlaying ? `${rpm} RPM` : '手动模式'}</span>
            </div>
          </div>

          {/* Tab Body */}
          <div className="p-3.5 max-h-[42vh] overflow-y-auto studio-scroll">
            {activeTab === 'build' && (
              <div className="grid grid-cols-1 md:grid-cols-12 gap-3.5">
                {/* Left: Draggable Modular Kit */}
                <div className="md:col-span-6 space-y-2">
                  <div className="flex items-center justify-between">
                    <h3 className="text-xs font-semibold text-[#1C1917]">
                      模块化积木库（拖拽至3D箱体或点击添加）
                    </h3>
                  </div>

                  <div className="grid grid-cols-2 sm:grid-cols-4 gap-1.5">
                    <button
                      type="button"
                      draggable
                      onDragStart={(e) => handleDragStart(e, 'cam-follower')}
                      onClick={() => onAddModule('cam-follower')}
                      className="flex flex-col items-start gap-0.5 p-1.5 text-left rounded-lg border border-[#1C1917]/12 bg-[#F3EFE6]/70 hover:bg-[#E5E0D5] hover:border-[#B45309]/50 transition-colors cursor-grab active:cursor-grabbing"
                    >
                      <div className="flex items-center justify-between w-full text-[#B45309]">
                        <Disc className="w-4 h-4" />
                        <Plus className="w-3.5 h-3.5" />
                      </div>
                      <span className="text-xs font-semibold text-[#1C1917] whitespace-nowrap">
                        凸轮从动组
                      </span>
                      <span className="text-[11px] text-[#57534E] leading-tight">
                        旋转转垂直往复
                      </span>
                    </button>

                    <button
                      type="button"
                      draggable
                      onDragStart={(e) => handleDragStart(e, 'four-bar-linkage')}
                      onClick={() => onAddModule('four-bar-linkage')}
                      className="flex flex-col items-start gap-0.5 p-1.5 text-left rounded-lg border border-[#1C1917]/12 bg-[#F3EFE6]/70 hover:bg-[#E5E0D5] hover:border-[#B45309]/50 transition-colors cursor-grab active:cursor-grabbing"
                    >
                      <div className="flex items-center justify-between w-full text-[#B45309]">
                        <GitCommitHorizontal className="w-4 h-4" />
                        <Plus className="w-3.5 h-3.5" />
                      </div>
                      <span className="text-xs font-semibold text-[#1C1917] whitespace-nowrap">
                        四连杆机构
                      </span>
                      <span className="text-[11px] text-[#57534E] leading-tight">
                        支点可拖曲柄摇杆
                      </span>
                    </button>

                    <button
                      type="button"
                      draggable
                      onDragStart={(e) => handleDragStart(e, 'bevel-gear')}
                      onClick={() => onAddModule('bevel-gear')}
                      className="flex flex-col items-start gap-0.5 p-1.5 text-left rounded-lg border border-[#1C1917]/12 bg-[#F3EFE6]/70 hover:bg-[#E5E0D5] hover:border-[#B45309]/50 transition-colors cursor-grab active:cursor-grabbing"
                    >
                      <div className="flex items-center justify-between w-full text-[#B45309]">
                        <Cog className="w-4 h-4" />
                        <Plus className="w-3.5 h-3.5" />
                      </div>
                      <span className="text-xs font-semibold text-[#1C1917] whitespace-nowrap">
                        90°锥齿轮
                      </span>
                      <span className="text-[11px] text-[#57534E] leading-tight">
                        水平转垂直变速
                      </span>
                    </button>

                    <button
                      type="button"
                      draggable
                      onDragStart={(e) => handleDragStart(e, 'pin-lantern')}
                      onClick={() => onAddModule('pin-lantern')}
                      className="flex flex-col items-start gap-0.5 p-1.5 text-left rounded-lg border border-[#1C1917]/12 bg-[#F3EFE6]/70 hover:bg-[#E5E0D5] hover:border-[#B45309]/50 transition-colors cursor-grab active:cursor-grabbing"
                    >
                      <div className="flex items-center justify-between w-full text-[#B45309]">
                        <CircleDot className="w-4 h-4" />
                        <Plus className="w-3.5 h-3.5" />
                      </div>
                      <span className="text-xs font-semibold text-[#1C1917] whitespace-nowrap">
                        冠形钉轮
                      </span>
                      <span className="text-[11px] text-[#57534E] leading-tight whitespace-nowrap">
                        90°灯笼针轮
                      </span>
                    </button>

                    <button
                      type="button"
                      draggable
                      onDragStart={(e) => handleDragStart(e, 'ratchet')}
                      onClick={() => onAddModule('ratchet')}
                      className="flex flex-col items-start gap-0.5 p-1.5 text-left rounded-lg border border-[#1C1917]/12 bg-[#F3EFE6]/70 hover:bg-[#E5E0D5] hover:border-[#B45309]/50 transition-colors cursor-grab active:cursor-grabbing"
                    >
                      <div className="flex items-center justify-between w-full text-[#B45309]">
                        <StepForward className="w-4 h-4" />
                        <Plus className="w-3.5 h-3.5" />
                      </div>
                      <span className="text-xs font-semibold text-[#1C1917] whitespace-nowrap">
                        棘轮机构
                      </span>
                      <span className="text-[11px] text-[#57534E] leading-tight whitespace-nowrap">
                        单向·机械步进
                      </span>
                    </button>

                    <button
                      type="button"
                      draggable
                      onDragStart={(e) => handleDragStart(e, 'circular-cam')}
                      onClick={() => onAddModule('circular-cam')}
                      className="flex flex-col items-start gap-0.5 p-1.5 text-left rounded-lg border border-[#1C1917]/12 bg-[#F3EFE6]/70 hover:bg-[#E5E0D5] hover:border-[#B45309]/50 transition-colors cursor-grab active:cursor-grabbing"
                    >
                      <div className="flex items-center justify-between w-full text-[#B45309]">
                        <Disc2 className="w-4 h-4" />
                        <Plus className="w-3.5 h-3.5" />
                      </div>
                      <span className="text-xs font-semibold text-[#1C1917] whitespace-nowrap">
                        90°圆形凸轮
                      </span>
                      <span className="text-[11px] text-[#57534E] leading-tight whitespace-nowrap">
                        正交圆盘·螺旋桨
                      </span>
                    </button>

                    <button
                      type="button"
                      draggable
                      onDragStart={(e) => handleDragStart(e, 'dual-eccentric')}
                      onClick={() => onAddModule('dual-eccentric')}
                      className="flex flex-col items-start gap-0.5 p-1.5 text-left rounded-lg border border-[#1C1917]/12 bg-[#F3EFE6]/70 hover:bg-[#E5E0D5] hover:border-[#B45309]/50 transition-colors cursor-grab active:cursor-grabbing"
                    >
                      <div className="flex items-center justify-between w-full text-[#B45309]">
                        <Repeat className="w-4 h-4" />
                        <Plus className="w-3.5 h-3.5" />
                      </div>
                      <span className="text-xs font-semibold text-[#1C1917] whitespace-nowrap">
                        偏心双凸轮
                      </span>
                      <span className="text-[11px] text-[#57534E] leading-tight whitespace-nowrap">
                        双轮交替·往复旋
                      </span>
                    </button>

                    <button
                      type="button"
                      draggable
                      onDragStart={(e) => handleDragStart(e, 'semi-circular-cam')}
                      onClick={() => onAddModule('semi-circular-cam')}
                      className="flex flex-col items-start gap-0.5 p-1.5 text-left rounded-lg border border-[#1C1917]/12 bg-[#F3EFE6]/70 hover:bg-[#E5E0D5] hover:border-[#B45309]/50 transition-colors cursor-grab active:cursor-grabbing"
                    >
                      <div className="flex items-center justify-between w-full text-[#B45309]">
                        <Contrast className="w-4 h-4" />
                        <Plus className="w-3.5 h-3.5" />
                      </div>
                      <span className="text-xs font-semibold text-[#1C1917] whitespace-nowrap">
                        正交半圆凸轮
                      </span>
                      <span className="text-[11px] text-[#57534E] leading-tight whitespace-nowrap">
                        半圆间歇·箭头
                      </span>
                    </button>
                  </div>

                  {/* Mounted Modules Selector */}
                  <div className="pt-1">
                    <div className="text-[11px] text-[#57534E] mb-1.5">
                      已装配轴上模块（点击选中，在3D中可直接左右拖拽）：
                    </div>
                    {blueprint.modules.length === 0 ? (
                      <div className="text-xs text-[#57534E] py-2 px-3 rounded-lg border border-dashed border-[#1C1917]/20">
                        当前主轴为空，请从上方添加凸轮或连杆积木。
                      </div>
                    ) : (
                      <div className="flex flex-wrap gap-1.5">
                        {blueprint.modules.map((m) => (
                          <button
                            key={m.id}
                            type="button"
                            onClick={() => onSelectModule(m.id)}
                            className={`px-2.5 py-1 text-xs font-medium rounded-md border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                              m.id === selectedModuleId
                                ? 'bg-[#1C1917] text-[#FAF8F4] border-[#1C1917]'
                                : 'bg-[#FAF8F4] text-[#1C1917] border-[#1C1917]/15 hover:bg-[#E5E0D5]'
                            }`}
                          >
                            {m.label}
                          </button>
                        ))}
                      </div>
                    )}
                  </div>
                </div>

                {/* Right: Selected Module Inspector */}
                <div className="md:col-span-6 md:border-l md:border-[#1C1917]/10 md:pl-4">
                  {selectedModule ? (
                    <div className="space-y-3">
                      <div className="flex items-center justify-between gap-2">
                        <div>
                          <h3 className="text-xs font-semibold text-[#1C1917]">
                            参数精调 · {selectedModule.label}
                          </h3>
                          <p className="text-[11px] text-[#57534E]">
                            {selectedModule.type === 'four-bar-linkage'
                              ? '提示：在3D场景中直接拖动红圈黄铜支点可实时改变连杆几何'
                              : selectedModule.type === 'circular-cam'
                                ? '提示：拖动红圈垂直凸轮可仅调托盘下位置（左/中/右），拖动托盘可整体移动'
                                : selectedModule.type === 'semi-circular-cam'
                                  ? '提示：半圆弧接触托盘时带动箭头步进，平切侧朝上时箭头精准停歇（可拖红圈调偏置）'
                                  : selectedModule.type === 'dual-eccentric'
                                    ? '提示：180°反相双偏心凸轮在单托盘下交替顶托并切换正反旋转方向'
                                    : '提示：在3D场景中左右拖拽组件可沿主轴滑动装配'}
                          </p>
                        </div>

                        <div className="flex items-center gap-3">
                          {/* Mini SVG Motion Curve */}
                          <svg
                            width="120"
                            height="36"
                            viewBox="0 0 120 36"
                            aria-label="位移运动曲线"
                            className="bg-[#F3EFE6] rounded border border-[#1C1917]/10 shrink-0"
                          >
                            <path
                              d={cyclePath}
                              fill="none"
                              stroke="#B45309"
                              strokeWidth="1.75"
                              strokeLinecap="round"
                            />
                          </svg>

                          <button
                            type="button"
                            onClick={() => onDeleteModule(selectedModule.id)}
                            aria-label={`移除 ${selectedModule.label}`}
                            className="p-1.5 text-[#57534E] hover:text-[#B91C1C] rounded-md hover:bg-[#FEE2E2]/60 transition-colors cursor-pointer"
                          >
                            <Trash2 className="w-4 h-4" />
                          </button>
                        </div>
                      </div>

                      <div className="grid grid-cols-2 sm:grid-cols-3 gap-x-4 gap-y-2.5 text-xs">
                        {/* Common: Slot X */}
                        <label className="flex flex-col gap-1">
                          <span className="flex justify-between text-[#57534E]">
                            <span>主轴位置 X</span>
                            <span className="font-mono-tabular text-[#1C1917]">
                              {selectedModule.slotX.toFixed(2)}
                            </span>
                          </span>
                          <input
                            type="range"
                            min={-2.55}
                            max={2.55}
                            step={0.05}
                            value={selectedModule.slotX}
                            onChange={(e) =>
                              onUpdateModule(selectedModule.id, {
                                slotX: parseFloat(e.target.value),
                              })
                            }
                            className="studio-slider"
                          />
                        </label>

                        {/* Common: Phase Offset */}
                        <label className="flex flex-col gap-1">
                          <span className="flex justify-between text-[#57534E]">
                            <span>相位角 Phase</span>
                            <span className="font-mono-tabular text-[#1C1917]">
                              {selectedModule.phaseDeg}°
                            </span>
                          </span>
                          <input
                            type="range"
                            min={0}
                            max={355}
                            step={5}
                            value={selectedModule.phaseDeg}
                            onChange={(e) =>
                              onUpdateModule(selectedModule.id, {
                                phaseDeg: parseInt(e.target.value, 10),
                              })
                            }
                            className="studio-slider"
                          />
                        </label>

                        {/* Common: Topper Sculpture */}
                        <label className="flex flex-col gap-1">
                          <span className="text-[#57534E]">舞台木偶造型</span>
                          <select
                            value={selectedModule.topper}
                            onChange={(e) =>
                              onUpdateModule(selectedModule.id, {
                                topper: e.target.value as TopperType,
                              })
                            }
                            className="px-2 py-1 text-xs bg-[#F3EFE6] border border-[#1C1917]/15 rounded-md text-[#1C1917]"
                          >
                            {TOPPER_LABELS.map((t) => (
                              <option key={t.id} value={t.id}>
                                {t.name}
                              </option>
                            ))}
                          </select>
                        </label>

                        {/* Cam-Follower Specific */}
                        {selectedModule.type === 'cam-follower' && (
                          <>
                            <label className="flex flex-col gap-1">
                              <span className="text-[#57534E]">凸轮轮廓</span>
                              <select
                                value={selectedModule.camType}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    camType: e.target.value as CamProfileType,
                                  })
                                }
                                className="px-2 py-1 text-xs bg-[#F3EFE6] border border-[#1C1917]/15 rounded-md text-[#1C1917]"
                              >
                                {CAM_LABELS.map((c) => (
                                  <option key={c.id} value={c.id}>
                                    {c.name}
                                  </option>
                                ))}
                              </select>
                            </label>

                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>升程 Lift</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  {selectedModule.lift.toFixed(2)}
                                </span>
                              </span>
                              <input
                                type="range"
                                min={0.18}
                                max={0.56}
                                step={0.02}
                                value={selectedModule.lift}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    lift: parseFloat(e.target.value),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>

                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>基圆半径</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  {selectedModule.baseRadius.toFixed(2)}
                                </span>
                              </span>
                              <input
                                type="range"
                                min={0.38}
                                max={0.64}
                                step={0.02}
                                value={selectedModule.baseRadius}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    baseRadius: parseFloat(e.target.value),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>
                          </>
                        )}

                        {/* 4-Bar Linkage Specific */}
                        {selectedModule.type === 'four-bar-linkage' && (
                          <>
                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>曲柄半径</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  {selectedModule.crankRadius.toFixed(2)}
                                </span>
                              </span>
                              <input
                                type="range"
                                min={0.25}
                                max={0.62}
                                step={0.02}
                                value={selectedModule.crankRadius}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    crankRadius: parseFloat(e.target.value),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>

                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>连杆长度</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  {selectedModule.couplerLength.toFixed(2)}
                                </span>
                              </span>
                              <input
                                type="range"
                                min={1.05}
                                max={2.1}
                                step={0.05}
                                value={selectedModule.couplerLength}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    couplerLength: parseFloat(e.target.value),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>

                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>摇杆长度</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  {selectedModule.rockerLength.toFixed(2)}
                                </span>
                              </span>
                              <input
                                type="range"
                                min={0.75}
                                max={1.55}
                                step={0.05}
                                value={selectedModule.rockerLength}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    rockerLength: parseFloat(e.target.value),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>
                          </>
                        )}

                        {/* Bevel Gear Specific */}
                        {selectedModule.type === 'bevel-gear' && (
                          <>
                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>传动速比</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  1 : {selectedModule.gearRatio.toFixed(1)}
                                </span>
                              </span>
                              <input
                                type="range"
                                min={0.5}
                                max={3.0}
                                step={0.5}
                                value={selectedModule.gearRatio}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    gearRatio: parseFloat(e.target.value),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>

                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>主动轮齿数</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  {selectedModule.teethCount}T
                                </span>
                              </span>
                              <input
                                type="range"
                                min={12}
                                max={24}
                                step={2}
                                value={selectedModule.teethCount}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    teethCount: parseInt(e.target.value, 10),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>
                          </>
                        )}

                        {/* Pin-Wheel & Lantern Pinion Specific */}
                        {selectedModule.type === 'pin-lantern' && (
                          <>
                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>传动速比</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  1 : {selectedModule.gearRatio.toFixed(1)}
                                </span>
                              </span>
                              <input
                                type="range"
                                min={0.5}
                                max={2.5}
                                step={0.5}
                                value={selectedModule.gearRatio}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    gearRatio: parseFloat(e.target.value),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>

                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>主轮插销数</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  {selectedModule.pegCount} 销
                                </span>
                              </span>
                              <input
                                type="range"
                                min={8}
                                max={16}
                                step={2}
                                value={selectedModule.pegCount}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    pegCount: parseInt(e.target.value, 10),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>
                          </>
                        )}

                        {/* Ratchet & Pawl Specific */}
                        {selectedModule.type === 'ratchet' && (
                          <>
                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>棘轮齿数</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  {selectedModule.teethCount} 齿
                                </span>
                              </span>
                              <input
                                type="range"
                                min={8}
                                max={16}
                                step={2}
                                value={selectedModule.teethCount}
                                onChange={(e) => {
                                  const nextTeeth = parseInt(e.target.value, 10);
                                  const maxStep = Math.max(1, Math.min(3, Math.floor(nextTeeth * 0.28)));
                                  onUpdateModule(selectedModule.id, {
                                    teethCount: nextTeeth,
                                    stepTeeth: Math.min(selectedModule.stepTeeth, maxStep),
                                  });
                                }}
                                className="studio-slider"
                              />
                            </label>

                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>单周步进</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  {selectedModule.stepTeeth} 齿 ({Math.round((selectedModule.stepTeeth * 360) / selectedModule.teethCount)}°)
                                </span>
                              </span>
                              <input
                                type="range"
                                min={1}
                                max={Math.max(1, Math.min(3, Math.floor(selectedModule.teethCount * 0.28)))}
                                step={1}
                                value={selectedModule.stepTeeth}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    stepTeeth: parseInt(e.target.value, 10),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>

                            <div className="flex flex-col gap-1 col-span-2 sm:col-span-3">
                              <span className="text-[#57534E]">单向锁定方向</span>
                              <div className="grid grid-cols-2 gap-1.5">
                                <button
                                  type="button"
                                  onClick={() =>
                                    onUpdateModule(selectedModule.id, { lockDirection: 1 })
                                  }
                                  className={`px-2.5 py-1.5 rounded-md text-xs font-medium border transition-colors cursor-pointer ${
                                    selectedModule.lockDirection === 1
                                      ? 'bg-[#1C1917] text-[#FAF8F4] border-[#1C1917]'
                                      : 'bg-[#F3EFE6] text-[#1C1917] border-[#1C1917]/12 hover:bg-[#E5E0D5]'
                                  }`}
                                >
                                  正转步进 · 逆止
                                </button>
                                <button
                                  type="button"
                                  onClick={() =>
                                    onUpdateModule(selectedModule.id, { lockDirection: -1 })
                                  }
                                  className={`px-2.5 py-1.5 rounded-md text-xs font-medium border transition-colors cursor-pointer ${
                                    selectedModule.lockDirection === -1
                                      ? 'bg-[#1C1917] text-[#FAF8F4] border-[#1C1917]'
                                      : 'bg-[#F3EFE6] text-[#1C1917] border-[#1C1917]/12 hover:bg-[#E5E0D5]'
                                  }`}
                                >
                                  反转步进 · 顺止
                                </button>
                              </div>
                            </div>
                          </>
                        )}

                        {/* 90-Degree Circular Cam Rotary Drive Specific */}
                        {selectedModule.type === 'circular-cam' && (() => {
                          const offset = selectedModule.camOffsetRatio ?? -1;
                          const camShape = selectedModule.camShape ?? 'circle';
                          const offsetStatus =
                            offset <= -0.98
                              ? '托盘最左侧 · 正转'
                              : Math.abs(offset) < 0.02
                                ? '托盘正中心 · 停转'
                                : offset >= 0.98
                                  ? '托盘最右侧 · 反转'
                                  : offset < 0
                                    ? `偏左 ${Math.round(Math.abs(offset) * 100)}% · 正转`
                                    : `偏右 ${Math.round(Math.abs(offset) * 100)}% · 反转`;

                          return (
                            <>
                              <label className="flex flex-col gap-1">
                                <span className="flex justify-between text-[#57534E]">
                                  <span>传动速比</span>
                                  <span className="font-mono-tabular text-[#1C1917]">
                                    1 : {selectedModule.gearRatio.toFixed(2)}
                                  </span>
                                </span>
                                <input
                                  type="range"
                                  min={0.5}
                                  max={3.0}
                                  step={0.25}
                                  value={selectedModule.gearRatio}
                                  onChange={(e) =>
                                    onUpdateModule(selectedModule.id, {
                                      gearRatio: parseFloat(e.target.value),
                                    })
                                  }
                                  className="studio-slider"
                                />
                              </label>

                              <label className="flex flex-col gap-1">
                                <span className="flex justify-between text-[#57534E]">
                                  <span>{camShape === 'circle' ? '圆盘半径' : '外接半径'}</span>
                                  <span className="font-mono-tabular text-[#1C1917]">
                                    {selectedModule.driverRadius.toFixed(2)}
                                  </span>
                                </span>
                                <input
                                  type="range"
                                  min={0.28}
                                  max={0.48}
                                  step={0.02}
                                  value={selectedModule.driverRadius}
                                  onChange={(e) =>
                                    onUpdateModule(selectedModule.id, {
                                      driverRadius: parseFloat(e.target.value),
                                    })
                                  }
                                  className="studio-slider"
                                />
                              </label>

                              <div className="flex flex-col gap-1.5 col-span-2 sm:col-span-3 pt-1 border-t border-[#1C1917]/10">
                                <div className="flex items-center justify-between text-[#57534E]">
                                  <span>凸轮轮形轮廓</span>
                                  <span className="font-mono-tabular text-[#1C1917]">
                                    {camShape === 'circle'
                                      ? '正圆形 · 恒速平转'
                                      : camShape === 'triangle'
                                        ? '正三角形 · 三拍起伏旋'
                                        : '正方形 · 四拍起伏旋'}
                                  </span>
                                </div>
                                <div className="grid grid-cols-3 gap-1.5" role="group" aria-label="凸轮轮形轮廓">
                                  {(
                                    [
                                      { id: 'circle', label: '正圆形' },
                                      { id: 'triangle', label: '正三角形' },
                                      { id: 'square', label: '正方形' },
                                    ] as const
                                  ).map((shapeOpt) => {
                                    const isActive = camShape === shapeOpt.id;
                                    return (
                                      <button
                                        key={shapeOpt.id}
                                        type="button"
                                        onClick={() =>
                                          onUpdateModule(selectedModule.id, {
                                            camShape: shapeOpt.id,
                                          })
                                        }
                                        className={`px-2 py-1.5 rounded-md text-xs font-medium border transition-colors cursor-pointer whitespace-nowrap ${
                                          isActive
                                            ? 'bg-[#1C1917] text-[#FAF8F4] border-[#1C1917]'
                                            : 'bg-[#F3EFE6] text-[#1C1917] border-[#1C1917]/12 hover:bg-[#E5E0D5]'
                                        }`}
                                      >
                                        {shapeOpt.label}
                                      </button>
                                    );
                                  })}
                                </div>
                              </div>

                              <div className="flex flex-col gap-1.5 col-span-2 sm:col-span-3 pt-1 border-t border-[#1C1917]/10">
                                <div className="flex items-center justify-between text-[#57534E]">
                                  <span>圆形凸轮相对托盘位置（仅移动凸轮盘）</span>
                                  <span className="font-mono-tabular font-medium text-[#B45309]">
                                    {offsetStatus}
                                  </span>
                                </div>
                                <div className="grid grid-cols-3 gap-1.5">
                                  {(
                                    [
                                      { label: '托盘最左侧', value: -1 },
                                      { label: '托盘正中心', value: 0 },
                                      { label: '托盘最右侧', value: 1 },
                                    ] as const
                                  ).map((presetPos) => {
                                    const isActive = Math.abs(offset - presetPos.value) < 0.03;
                                    return (
                                      <button
                                        key={presetPos.label}
                                        type="button"
                                        onClick={() =>
                                          onUpdateModule(selectedModule.id, {
                                            camOffsetRatio: presetPos.value,
                                          })
                                        }
                                        className={`px-2 py-1.5 rounded-md text-xs font-medium border transition-colors cursor-pointer whitespace-nowrap ${
                                          isActive
                                            ? 'bg-[#1C1917] text-[#FAF8F4] border-[#1C1917]'
                                            : 'bg-[#F3EFE6] text-[#1C1917] border-[#1C1917]/12 hover:bg-[#E5E0D5]'
                                        }`}
                                      >
                                        {presetPos.label}
                                      </button>
                                    );
                                  })}
                                </div>
                                <input
                                  type="range"
                                  aria-label="圆形凸轮相对托盘水平位置"
                                  min={-1}
                                  max={1}
                                  step={0.05}
                                  value={offset}
                                  onChange={(e) =>
                                    onUpdateModule(selectedModule.id, {
                                      camOffsetRatio: parseFloat(e.target.value),
                                    })
                                  }
                                  className="studio-slider"
                                />
                              </div>
                            </>
                          );
                        })()}

                        {/* Dual Eccentric Alternating Drive Specific */}
                        {selectedModule.type === 'dual-eccentric' && (
                          <>
                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>摆转速比</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  1 : {selectedModule.gearRatio.toFixed(2)}
                                </span>
                              </span>
                              <input
                                type="range"
                                min={0.5}
                                max={2.5}
                                step={0.25}
                                value={selectedModule.gearRatio}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    gearRatio: parseFloat(e.target.value),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>

                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>圆盘半径</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  {selectedModule.camRadius.toFixed(2)}
                                </span>
                              </span>
                              <input
                                type="range"
                                min={0.28}
                                max={0.4}
                                step={0.02}
                                value={selectedModule.camRadius}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    camRadius: parseFloat(e.target.value),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>

                            <label className="flex flex-col gap-1">
                              <span className="flex justify-between text-[#57534E]">
                                <span>偏心距</span>
                                <span className="font-mono-tabular text-[#1C1917]">
                                  {selectedModule.eccentricity.toFixed(2)}
                                </span>
                              </span>
                              <input
                                type="range"
                                min={0.04}
                                max={0.12}
                                step={0.01}
                                value={selectedModule.eccentricity}
                                onChange={(e) =>
                                  onUpdateModule(selectedModule.id, {
                                    eccentricity: parseFloat(e.target.value),
                                  })
                                }
                                className="studio-slider"
                              />
                            </label>
                          </>
                        )}

                        {/* 90-Degree Semicircular Cam Intermittent Rotary Drive Specific */}
                        {selectedModule.type === 'semi-circular-cam' && (() => {
                          const offset = selectedModule.camOffsetRatio ?? -1;
                          const stepDeg = Math.round(180 * selectedModule.gearRatio * Math.abs(offset));
                          const offsetStatus =
                            offset <= -0.98
                              ? `托盘最左侧 · 正转 ${stepDeg}°/圈`
                              : Math.abs(offset) < 0.02
                                ? '托盘正中心 · 停转'
                                : offset >= 0.98
                                  ? `托盘最右侧 · 反转 ${stepDeg}°/圈`
                                  : offset < 0
                                    ? `偏左 ${Math.round(Math.abs(offset) * 100)}% · 正转`
                                    : `偏右 ${Math.round(Math.abs(offset) * 100)}% · 反转`;

                          return (
                            <>
                              <label className="flex flex-col gap-1">
                                <span className="flex justify-between text-[#57534E]">
                                  <span>步进转速比</span>
                                  <span className="font-mono-tabular text-[#1C1917]">
                                    1 : {selectedModule.gearRatio.toFixed(2)}
                                  </span>
                                </span>
                                <input
                                  type="range"
                                  min={0.5}
                                  max={2.5}
                                  step={0.25}
                                  value={selectedModule.gearRatio}
                                  onChange={(e) =>
                                    onUpdateModule(selectedModule.id, {
                                      gearRatio: parseFloat(e.target.value),
                                    })
                                  }
                                  className="studio-slider"
                                />
                              </label>

                              <label className="flex flex-col gap-1">
                                <span className="flex justify-between text-[#57534E]">
                                  <span>半圆半径</span>
                                  <span className="font-mono-tabular text-[#1C1917]">
                                    {selectedModule.driverRadius.toFixed(2)}
                                  </span>
                                </span>
                                <input
                                  type="range"
                                  min={0.28}
                                  max={0.46}
                                  step={0.02}
                                  value={selectedModule.driverRadius}
                                  onChange={(e) =>
                                    onUpdateModule(selectedModule.id, {
                                      driverRadius: parseFloat(e.target.value),
                                    })
                                  }
                                  className="studio-slider"
                                />
                              </label>

                              <div className="flex flex-col gap-1.5 col-span-2 sm:col-span-3 pt-1 border-t border-[#1C1917]/10">
                                <div className="flex items-center justify-between text-[#57534E]">
                                  <span>半圆凸轮相对托盘位置（仅移动凸轮）</span>
                                  <span className="font-mono-tabular font-medium text-[#B45309]">
                                    {offsetStatus}
                                  </span>
                                </div>
                                <div className="grid grid-cols-3 gap-1.5">
                                  {(
                                    [
                                      { label: '托盘最左侧', value: -1 },
                                      { label: '托盘正中心', value: 0 },
                                      { label: '托盘最右侧', value: 1 },
                                    ] as const
                                  ).map((presetPos) => {
                                    const isActive = Math.abs(offset - presetPos.value) < 0.03;
                                    return (
                                      <button
                                        key={presetPos.label}
                                        type="button"
                                        onClick={() =>
                                          onUpdateModule(selectedModule.id, {
                                            camOffsetRatio: presetPos.value,
                                          })
                                        }
                                        className={`px-2 py-1.5 rounded-md text-xs font-medium border transition-colors cursor-pointer whitespace-nowrap ${
                                          isActive
                                            ? 'bg-[#1C1917] text-[#FAF8F4] border-[#1C1917]'
                                            : 'bg-[#F3EFE6] text-[#1C1917] border-[#1C1917]/12 hover:bg-[#E5E0D5]'
                                        }`}
                                      >
                                        {presetPos.label}
                                      </button>
                                    );
                                  })}
                                </div>
                                <input
                                  type="range"
                                  aria-label="半圆凸轮相对托盘水平位置"
                                  min={-1}
                                  max={1}
                                  step={0.05}
                                  value={offset}
                                  onChange={(e) =>
                                    onUpdateModule(selectedModule.id, {
                                      camOffsetRatio: parseFloat(e.target.value),
                                    })
                                  }
                                  className="studio-slider"
                                />
                              </div>
                            </>
                          );
                        })()}
                      </div>
                    </div>
                  ) : (
                    <div className="h-full flex flex-col justify-center items-start gap-1 py-3 text-xs text-[#57534E]">
                      <span className="font-semibold text-[#1C1917]">未选中任何机械积木</span>
                      <span>
                        请在3D视图中点击凸轮、连杆或齿轮组件，或从左侧积木库拖拽新组件到主轴上。
                      </span>
                    </div>
                  )}
                </div>
              </div>
            )}

            {activeTab === 'motion' && (
              <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-4 gap-4 text-xs">
                <div className="space-y-2">
                  <div className="flex justify-between text-[#57534E]">
                    <span>曲柄转速</span>
                    <span className="font-mono-tabular text-[#1C1917]">{rpm} RPM</span>
                  </div>
                  <input
                    type="range"
                    min={6}
                    max={60}
                    step={2}
                    value={rpm}
                    onChange={(e) => onChangeRpm(parseInt(e.target.value, 10))}
                    className="studio-slider w-full"
                  />
                  <button
                    type="button"
                    onClick={onTogglePlay}
                    className="w-full py-1.5 px-3 rounded-md border border-[#1C1917]/15 bg-[#F3EFE6] hover:bg-[#E5E0D5] font-medium text-[#1C1917] cursor-pointer"
                  >
                    {isPlaying ? '切换为手动摇柄检验' : '恢复连续旋转'}
                  </button>
                </div>

                <div className="space-y-2">
                  <div className="flex justify-between text-[#57534E]">
                    <span>手动摇柄角度</span>
                    <span className="font-mono-tabular text-[#1C1917]">{manualAngleDeg}°</span>
                  </div>
                  <input
                    type="range"
                    min={0}
                    max={359}
                    step={1}
                    value={manualAngleDeg}
                    onChange={(e) => onChangeManualAngle(parseInt(e.target.value, 10))}
                    className="studio-slider w-full"
                  />
                  <p className="text-[11px] text-[#57534E]">
                    拖动角度尺可逐度检查凸轮升程与连杆死点
                  </p>
                </div>

                <div className="space-y-2">
                  <div className="flex justify-between text-[#57534E]">
                    <span>透明橱窗剖视</span>
                    <span className="font-mono-tabular text-[#1C1917]">
                      {Math.round(cutawayAmount * 100)}%
                    </span>
                  </div>
                  <input
                    type="range"
                    min={0}
                    max={1}
                    step={0.05}
                    value={cutawayAmount}
                    onChange={(e) => onChangeCutaway(parseFloat(e.target.value))}
                    className="studio-slider w-full"
                  />
                  <div className="flex justify-between text-[#57534E] pt-1">
                    <span>箱体爆炸分解</span>
                    <span className="font-mono-tabular text-[#1C1917]">
                      {Math.round(explodeAmount * 100)}%
                    </span>
                  </div>
                  <input
                    type="range"
                    min={0}
                    max={1}
                    step={0.05}
                    value={explodeAmount}
                    onChange={(e) => onChangeExplode(parseFloat(e.target.value))}
                    className="studio-slider w-full"
                  />
                </div>

                <div className="space-y-2">
                  <span className="block text-[#57534E]">机位视角预设</span>
                  <div className="grid grid-cols-2 gap-1.5">
                    {CAMERA_OPTIONS.map((cam) => (
                      <button
                        key={cam.id}
                        type="button"
                        onClick={() => onSelectCameraPreset(cam.id)}
                        className={`px-2 py-1.5 rounded-md text-xs font-medium border transition-colors whitespace-nowrap cursor-pointer ${
                          cameraPreset === cam.id
                            ? 'bg-[#1C1917] text-[#FAF8F4] border-[#1C1917]'
                            : 'bg-[#F3EFE6] text-[#1C1917] border-[#1C1917]/12 hover:bg-[#E5E0D5]'
                        }`}
                      >
                        {cam.name}
                      </button>
                    ))}
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'presets' && (
              <div className="grid grid-cols-1 md:grid-cols-12 gap-4 text-xs">
                <div className="md:col-span-4 space-y-2">
                  <span className="block font-semibold text-[#1C1917]">木作与五金涂装</span>
                  <div className="space-y-1.5">
                    {FINISH_OPTIONS.map((f) => (
                      <button
                        key={f.id}
                        type="button"
                        onClick={() => onChangeFinish(f.id)}
                        className={`w-full flex items-center justify-between px-3 py-2 rounded-lg border transition-colors cursor-pointer ${
                          blueprint.finish === f.id
                            ? 'bg-[#1C1917] text-[#FAF8F4] border-[#1C1917]'
                            : 'bg-[#F3EFE6] text-[#1C1917] border-[#1C1917]/12 hover:bg-[#E5E0D5]'
                        }`}
                      >
                        <span className="font-medium">{f.name}</span>
                        <span className="flex items-center gap-1">
                          <span
                            className="w-3.5 h-3.5 rounded-full border border-white/40"
                            style={{ backgroundColor: f.woodHex }}
                          />
                          <span
                            className="w-3.5 h-3.5 rounded-full border border-white/40"
                            style={{ backgroundColor: f.metalHex }}
                          />
                        </span>
                      </button>
                    ))}
                  </div>
                </div>

                <div className="md:col-span-8 md:border-l md:border-[#1C1917]/10 md:pl-4 space-y-2">
                  <div className="flex items-center justify-between">
                    <span className="font-semibold text-[#1C1917]">
                      经典 Automata 样板库（参考 Cabaret Mechanical Theatre）
                    </span>
                    <button
                      type="button"
                      onClick={onClearFrame}
                      className="text-xs font-medium text-[#B45309] hover:underline cursor-pointer"
                    >
                      载入空白木框
                    </button>
                  </div>
                  <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
                    {CLASSIC_PRESETS.map((preset) => (
                      <button
                        key={preset.id}
                        type="button"
                        onClick={() => onLoadPreset(preset.id)}
                        className={`text-left p-2.5 rounded-lg border transition-colors cursor-pointer ${
                          blueprint.id === preset.id
                            ? 'bg-[#1C1917] text-[#FAF8F4] border-[#1C1917]'
                            : 'bg-[#F3EFE6]/80 text-[#1C1917] border-[#1C1917]/12 hover:bg-[#E5E0D5]'
                        }`}
                      >
                        <div className="font-semibold truncate">{preset.name}</div>
                        <div
                          className={`text-[11px] truncate mt-0.5 ${
                            blueprint.id === preset.id ? 'text-[#D6D3D1]' : 'text-[#57534E]'
                          }`}
                        >
                          {preset.styleCredit}
                        </div>
                      </button>
                    ))}
                  </div>
                </div>
              </div>
            )}
          </div>
        </section>
      )}
    </div>
  );
};
