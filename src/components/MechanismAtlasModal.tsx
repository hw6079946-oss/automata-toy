import React, { useMemo } from 'react';
import { X } from 'lucide-react';
import { sampleCamContour, sampleCycleCurve } from '../engine/kinematics';
import { CLASSIC_PRESETS } from '../engine/presets';
import type { CamProfileType } from '../engine/types';

interface MechanismAtlasModalProps {
  open: boolean;
  onClose: () => void;
  onLoadPreset: (presetId: string) => void;
}

const CAM_ARCHETYPES: Array<{
  type: CamProfileType;
  title: string;
  subtitle: string;
  desc: string;
}> = [
  {
    type: 'eccentric',
    title: '偏心圆凸轮 (Eccentric Cam)',
    subtitle: '正弦简谐运动 · 无停顿',
    desc: '轴孔偏离圆心的圆盘，每转一周产生一次柔和的正弦升降，常用于浮标、海浪或呼吸式胸腔起伏。',
  },
  {
    type: 'pear',
    title: '蛋形 / 梨形凸轮 (Pear / Egg Cam)',
    subtitle: '半周静止 · 半周起落',
    desc: 'Cabaret Mechanical Theatre 最经典的造型：下半圆贴近基圆，木偶在低位静止蓄势，凸瓣经过时优雅振翅。',
  },
  {
    type: 'snail',
    title: '蜗牛骤降凸轮 (Snail / Drop Cam)',
    subtitle: '等速爬升 · 瞬间跌落',
    desc: '阿基米德螺线缓慢抬升顶杆，随后在台阶处骤然释放落差，适合青蛙突跳、敲锤或惊吓弹出动作。',
  },
  {
    type: 'heart',
    title: '心形双峰凸轮 (Double-Lobe Cam)',
    subtitle: '一转双动 · 节奏加倍',
    desc: '单次曲柄旋转驱动两次完整的起伏节拍，常用于振翅频率加倍或对称双节拍机械舞者。',
  },
];

export const MechanismAtlasModal: React.FC<MechanismAtlasModalProps> = ({
  open,
  onClose,
  onLoadPreset,
}) => {
  const camDiagrams = useMemo(() => {
    return CAM_ARCHETYPES.map((item) => {
      const contour = sampleCamContour(item.type, 16, 12, 64);
      const contourPath =
        contour
          .map((p, i) => `${i === 0 ? 'M' : 'L'}${(36 + p.x).toFixed(1)},${(36 - p.y).toFixed(1)}`)
          .join(' ') + ' Z';
      const curve = sampleCycleCurve(
        {
          id: 'atlas',
          type: 'cam-follower',
          label: '',
          slotX: 0,
          phaseDeg: 0,
          camType: item.type,
          baseRadius: 0.55,
          lift: 0.48,
          footRadius: 0.32,
          topper: 'none',
          topperScale: 1,
        },
        48
      );
      const wavePath = curve
        .map((v, i) => `${i === 0 ? 'M' : 'L'}${(i / 47) * 110},${34 - v * 26}`)
        .join(' ');
      return { ...item, contourPath, wavePath };
    });
  }, []);

  if (!open) return null;

  return (
    <div
      role="dialog"
      aria-modal="true"
      aria-labelledby="atlas-modal-title"
      className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-[#1C1917]/45"
    >
      <div className="w-full max-w-4xl max-h-[85vh] flex flex-col bg-[#FAF8F4] border border-[#1C1917]/15 rounded-xl shadow-2xl overflow-hidden">
        <div className="flex items-center justify-between px-6 py-4 border-b border-[#1C1917]/12 bg-[#F3EFE6]">
          <div>
            <h2 id="atlas-modal-title" className="font-display text-xl font-semibold text-[#1C1917]">
              经典 Automata 机械结构图谱
            </h2>
            <p className="text-xs text-[#57534E] mt-0.5">
              借鉴 Cabaret Mechanical Theatre (CMT) 与 Paul Spooner 传统木构自动木偶机理
            </p>
          </div>
          <button
            type="button"
            onClick={onClose}
            aria-label="关闭图谱"
            className="p-2 rounded-lg text-[#57534E] hover:text-[#1C1917] hover:bg-[#E5E0D5] transition-colors cursor-pointer"
          >
            <X className="w-5 h-5" />
          </button>
        </div>

        <div className="p-6 overflow-y-auto studio-scroll space-y-6">
          {/* 1. Classic Cam Archetypes */}
          <section className="space-y-3">
            <h3 className="text-sm font-semibold text-[#1C1917]">
              01. 四大经典凸轮轮廓与位移曲线
            </h3>
            <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
              {camDiagrams.map((c) => (
                <div
                  key={c.type}
                  className="flex items-start gap-4 p-3.5 rounded-lg border border-[#1C1917]/12 bg-[#F3EFE6]/50"
                >
                  <div className="flex flex-col items-center gap-1.5 shrink-0">
                    <svg width="72" height="72" viewBox="0 0 72 72" aria-hidden="true">
                      <path
                        d={c.contourPath}
                        fill="#D8B589"
                        stroke="#1C1917"
                        strokeWidth="1.5"
                      />
                      <circle cx="36" cy="36" r="4" fill="#B45309" />
                    </svg>
                    <svg width="110" height="38" viewBox="0 0 110 38" aria-hidden="true">
                      <line x1="0" y1="34" x2="110" y2="34" stroke="rgba(28,25,23,0.15)" />
                      <path
                        d={c.wavePath}
                        fill="none"
                        stroke="#B45309"
                        strokeWidth="1.8"
                      />
                    </svg>
                  </div>
                  <div className="space-y-1">
                    <div className="text-xs font-semibold text-[#1C1917]">{c.title}</div>
                    <div className="text-[11px] font-medium text-[#B45309]">{c.subtitle}</div>
                    <p className="text-xs text-[#57534E] leading-relaxed">{c.desc}</p>
                  </div>
                </div>
              ))}
            </div>
          </section>

          {/* 2. Linkages, 90-deg Drives & Ratchet */}
          <section className="grid grid-cols-1 sm:grid-cols-2 gap-4 pt-2 border-t border-[#1C1917]/10">
            <div className="space-y-1.5">
              <h3 className="text-sm font-semibold text-[#1C1917]">
                02. 曲柄摇杆四连杆机构 (4-Bar Crank-Rocker)
              </h3>
              <p className="text-xs text-[#57534E] leading-relaxed">
                将主轴的连续圆周旋转转换为上方木偶的往复俯仰或水平划桨动作。在 3D 视图中可直接拖动
                <strong className="text-[#1C1917]">红环标记的黄铜支点</strong>，实时调整摇杆支点高度 (Y) 与前后偏置 (Z)。
              </p>
            </div>
            <div className="space-y-1.5">
              <h3 className="text-sm font-semibold text-[#1C1917]">
                03. 90° 冠形锥齿轮传动 (Bevel Gear)
              </h3>
              <p className="text-xs text-[#57534E] leading-relaxed">
                通过共锥顶渐开线斜齿啮合，将水平旋转平稳变为垂直轴自转或舞台旋转风车，支持 0.5x 至 3.0x 精确变速。
              </p>
            </div>
            <div className="space-y-1.5">
              <h3 className="text-sm font-semibold text-[#1C1917]">
                04. 冠形钉轮与灯笼针轮 (Pin-Wheel &amp; Lantern Pinion)
              </h3>
              <p className="text-xs text-[#57534E] leading-relaxed">
                经典木工风车与达芬奇手稿常用的 90° 锥齿轮替代机构：圆盘轴向黄铜插销拨动悬臂开口式灯笼针轮竖轴，无需精密锥齿加工且自带抗偏心容差。
              </p>
            </div>
            <div className="space-y-1.5">
              <h3 className="text-sm font-semibold text-[#1C1917]">
                05. 棘轮单向步进机构 (Ratchet &amp; Pawl)
              </h3>
              <p className="text-xs text-[#57534E] leading-relaxed">
                曲柄连杆驱动摆臂上的主动棘爪单向拨动锯齿棘轮，回程时由机架上的止动棘爪咬合齿根锁定，将连续转动转化为清晰的「步进—停顿」间歇节拍，并支持正反单向切换。
              </p>
            </div>
            <div className="space-y-1.5">
              <h3 className="text-sm font-semibold text-[#1C1917]">
                06. 90° 圆形凸轮正交旋转机构 (90° Circular Cam Rotary Drive)
              </h3>
              <p className="text-xs text-[#57534E] leading-relaxed">
                主轴上的垂直凸轮盘以其高摩擦边缘切向抵接上方水平转盘底面，通过纯滚动正交摩擦将水平旋转转为垂直旋转输出，默认驱动舞台上的四叶十字螺旋桨平稳旋转。支持切换「正圆形 / 正三角形 / 正方形」凸轮轮廓（三拍/四拍起伏变速），并支持整体沿主轴移动及单独调整凸轮在托盘下的左/中/右位置。
              </p>
            </div>
            <div className="space-y-1.5">
              <h3 className="text-sm font-semibold text-[#1C1917]">
                07. 偏心双凸轮交替驱动机构 (Twin Eccentric Alternating Drive)
              </h3>
              <p className="text-xs text-[#57534E] leading-relaxed">
                在同一水平托盘下方并排装配一对相位差 180° 的偏心圆形凸轮：前半周左偏心轮顶起托盘并摩擦驱动其正向旋转（右轮自然悬空脱离）；后半周右偏心轮升起接替顶托，令托盘与上方木偶平稳反向旋转，形成左右交替的往复回转与微幅起伏节拍。
              </p>
            </div>
            <div className="space-y-1.5">
              <h3 className="text-sm font-semibold text-[#1C1917]">
                08. 正交半圆凸轮间歇旋转机构 (Semi-Circular Cam Intermittent Drive)
              </h3>
              <p className="text-xs text-[#57534E] leading-relaxed">
                主轴上的垂直半圆凸轮具有 180° 圆弧摩擦工作段与 180° 平切退让空行程：圆弧段切向贴合托盘底面时驱动立轴与舞台指示箭头顺畅旋转；平切侧转至上方时与托盘脱开形成清晰间隙，令箭头精确停歇驻留，从而把连续旋转转换为清晰的「旋转—停顿」间歇步进动作。
              </p>
            </div>
          </section>

          {/* 3. Quick Load Classic Presets */}
          <section className="space-y-2.5 pt-2 border-t border-[#1C1917]/10">
            <h3 className="text-sm font-semibold text-[#1C1917]">09. 经典样板一键加载</h3>
            <div className="grid grid-cols-1 sm:grid-cols-2 gap-2.5">
              {CLASSIC_PRESETS.map((preset) => (
                <button
                  key={preset.id}
                  type="button"
                  onClick={() => {
                    onLoadPreset(preset.id);
                    onClose();
                  }}
                  className="text-left p-3 rounded-lg border border-[#1C1917]/12 bg-[#F3EFE6]/70 hover:bg-[#E5E0D5] transition-colors cursor-pointer"
                >
                  <div className="text-xs font-semibold text-[#1C1917]">{preset.name}</div>
                  <div className="text-[11px] text-[#B45309] mt-0.5">{preset.styleCredit}</div>
                  <p className="text-xs text-[#57534E] mt-1 leading-relaxed">{preset.description}</p>
                </button>
              ))}
            </div>
          </section>
        </div>
      </div>
    </div>
  );
};
