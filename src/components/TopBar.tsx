import React from 'react';
import { Pause, Play } from 'lucide-react';

interface TopBarProps {
  isPlaying: boolean;
  onTogglePlay: () => void;
  onOpenAtlas: () => void;
  onSelectPresetsTab: () => void;
  onClearFrame: () => void;
  onExportBlueprint: () => void;
}

export const TopBar: React.FC<TopBarProps> = ({
  isPlaying,
  onTogglePlay,
  onOpenAtlas,
  onSelectPresetsTab,
  onClearFrame,
  onExportBlueprint,
}) => {
  return (
    <header className="relative z-20 flex items-center justify-between gap-8 px-6 py-3.5 bg-[#FAF8F4] border-b border-[#1C1917]/12">
      {/* Zone 1: Single text element wordmark */}
      <a
        href="#top"
        onClick={(e) => {
          e.preventDefault();
          onSelectPresetsTab();
        }}
        className="font-display text-lg font-semibold tracking-tight text-[#1C1917] whitespace-nowrap shrink-0"
      >
        Atelier Automaton
      </a>

      {/* Zone 2: Concise single-line nav links */}
      <nav
        aria-label="主导航"
        className="hidden md:flex items-center gap-6 text-sm font-medium text-[#57534E]"
      >
        <button
          type="button"
          onClick={onSelectPresetsTab}
          className="hover:text-[#1C1917] hover:underline underline-offset-4 transition-colors whitespace-nowrap shrink-0 cursor-pointer"
        >
          经典范例
        </button>
        <button
          type="button"
          onClick={onOpenAtlas}
          className="hover:text-[#1C1917] hover:underline underline-offset-4 transition-colors whitespace-nowrap shrink-0 cursor-pointer"
        >
          机理图谱
        </button>
        <button
          type="button"
          onClick={onClearFrame}
          className="hover:text-[#1C1917] hover:underline underline-offset-4 transition-colors whitespace-nowrap shrink-0 cursor-pointer"
        >
          清空画板
        </button>
        <button
          type="button"
          onClick={onExportBlueprint}
          className="hover:text-[#1C1917] hover:underline underline-offset-4 transition-colors whitespace-nowrap shrink-0 cursor-pointer"
        >
          导出蓝图
        </button>
      </nav>

      {/* Zone 3: Single primary action */}
      <div className="flex items-center shrink-0">
        <button
          type="button"
          onClick={onTogglePlay}
          className="inline-flex items-center gap-2 px-4 py-2 text-xs font-semibold text-[#FAF8F4] bg-[#B45309] hover:bg-[#92400E] rounded-lg transition-colors whitespace-nowrap shrink-0 cursor-pointer focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-[#B45309]"
        >
          {isPlaying ? <Pause className="w-3.5 h-3.5" /> : <Play className="w-3.5 h-3.5" />}
          <span>{isPlaying ? '暂停运转' : '启动曲柄'}</span>
        </button>
      </div>
    </header>
  );
};
