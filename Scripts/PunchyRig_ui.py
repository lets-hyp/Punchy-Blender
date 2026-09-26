import bpy
import json
import math
import mathutils
import os
import re
from bpy_extras.io_utils import ExportHelper, ImportHelper


# ==========================================
# 1. PROPERTY GROUPS
# ==========================================
class HypActionExportItem(bpy.types.PropertyGroup):
    action_name: bpy.props.StringProperty()
    export: bpy.props.BoolProperty(default=True)
    # exportMeta.json senkronizasyonu: bu action hangi gruba ait
    # (meta'da hiçbir grupta kayıtlı değilse "Unlisted")
    group_name: bpy.props.StringProperty(default="Unlisted")


def _hyp_group_select_all_update(self, context):
    """Grup başlığındaki tek kutucuk: bu gruba ait tüm action'ların
    export checkbox'ını toplu olarak aç/kapat."""
    for item in context.scene.hyp_export_list:
        if item.group_name == self.group_name:
            item.export = self.select_all


class HypExportGroupItem(bpy.types.PropertyGroup):
    group_name: bpy.props.StringProperty()
    select_all: bpy.props.BoolProperty(default=False, update=_hyp_group_select_all_update)


# ==========================================
# 2. KEYFRAME UTILITIES
# ==========================================
def clean_keyframes(key_dict, tolerance=0.001):
    if not key_dict: return {}
    times = sorted([float(k) for k in key_dict.keys()])
    if len(times) <= 2: return key_dict
    cleaned = {str(times[0]): key_dict[str(times[0])]}
    last_kept_time = times[0]
    for i in range(1, len(times) - 1):
        prev_val = key_dict[str(last_kept_time)]
        curr_val = key_dict[str(times[i])]
        next_val = key_dict[str(times[i+1])]
        diff_prev = sum(abs(curr_val[j] - prev_val[j]) for j in range(3))
        diff_next = sum(abs(next_val[j] - curr_val[j]) for j in range(3))
        if diff_prev > tolerance or diff_next > tolerance:
            cleaned[str(times[i])] = curr_val
            last_kept_time = times[i]
    cleaned[str(times[-1])] = key_dict[str(times[-1])]
    return cleaned


def _rdp_reduce(times, values, epsilon):
    """Ramer-Douglas-Peucker: bir segmentin iki uc noktasini birlestiren
    dogru parcasindan epsilon'dan fazla sapan en uzak ara noktayi bulur;
    bulursa segmenti o noktadan ikiye bolup rekursif devam eder, yoksa
    sadece iki uc noktayi birakir. times/values ayni sirada eslesen,
    zaten zaman siralamasina gore sirali listelerdir."""
    if len(times) < 3:
        return list(times), list(values)

    t0, t1 = times[0], times[-1]
    v0, v1 = values[0], values[-1]
    dt = t1 - t0

    max_dist = -1.0
    max_idx = 0
    for i in range(1, len(times) - 1):
        if dt == 0:
            interp = v0
        else:
            frac = (times[i] - t0) / dt
            interp = [v0[j] + frac * (v1[j] - v0[j]) for j in range(len(v0))]
        # O andaki gercek deger ile "duz cizgi" varsayiminin tahmin ettigi
        # deger arasindaki toplam sapma (eksenler toplanarak, clean_keyframes
        # ile ayni olcum mantigi kullanilir).
        dist = sum(abs(values[i][j] - interp[j]) for j in range(len(v0)))
        if dist > max_dist:
            max_dist = dist
            max_idx = i

    if max_dist > epsilon:
        left_t, left_v = _rdp_reduce(times[:max_idx + 1], values[:max_idx + 1], epsilon)
        right_t, right_v = _rdp_reduce(times[max_idx:], values[max_idx:], epsilon)
        # max_idx noktasi hem sol hem sag parcada var; sol parcanin sonunu
        # atarak cift eklenmesini onluyoruz.
        return left_t[:-1] + right_t, left_v[:-1] + right_v

    return [times[0], times[-1]], [values[0], values[-1]]


def simplify_keyframes(key_dict, epsilon=0.05):
    """clean_keyframes'ten SONRA calisir. clean_keyframes zaten tamamen
    sabit/duz bolgeleri temizliyor; bu fonksiyon ise onun temizleyemedigi
    yumusak/egrisel hareket eden bolgelerdeki gereksiz ara key'leri RDP
    toleransina (epsilon) gore sadelestirir. Ilk ve son key HER ZAMAN
    korunur, animasyonun baslangic/bitis pozu asla degismez."""
    if not key_dict or len(key_dict) <= 2:
        return key_dict

    times = sorted(float(k) for k in key_dict.keys())
    values = [key_dict[str(t)] for t in times]

    reduced_times, reduced_values = _rdp_reduce(times, values, epsilon)

    return {str(t): v for t, v in zip(reduced_times, reduced_values)}


def _quat_to_xyz_euler_deg(q: mathutils.Quaternion):
    w, x, y, z = q.w, q.x, q.y, q.z

    m02 =  2.0 * (x*z + w*y)
    m12 =  2.0 * (y*z - w*x)
    m22 =  1.0 - 2.0 * (x*x + y*y)
    m01 =  2.0 * (x*y - w*z)
    m00 =  1.0 - 2.0 * (y*y + z*z)

    sin_y = max(-1.0, min(1.0, m02))
    theta_y = math.asin(sin_y)

    SINGULAR = 1.0 - 1e-6

    if abs(sin_y) < SINGULAR:
        theta_x = math.atan2(-m12, m22)
        theta_z = math.atan2(-m01, m00)
    else:
        m10 =  2.0 * (x*y + w*z)
        m11 =  1.0 - 2.0 * (x*x + z*z)

        theta_x = math.atan2(m10 if sin_y > 0.0 else -m10, m11)
        theta_z = 0.0

    return (math.degrees(theta_x),
            math.degrees(theta_y),
            math.degrees(theta_z))


def _unwrap_euler(new_degs, prev_degs):
    return [
        new - 360.0 * math.floor((new - prev + 180.0) / 360.0)
        for new, prev in zip(new_degs, prev_degs)
    ]


# ==========================================
# 2b. METADATA TIMELINE (RAW ACTION CURVE) UTILITIES
# ==========================================
_TIMELINE_CHANNELS = (
    ("location", 3),
    ("rotation_euler", 3),
    ("rotation_quaternion", 4),
    ("rotation_axis_angle", 4),
    ("scale", 3),
)

_TIMELINE_ROTATION_MODE = {
    "rotation_euler": "XYZ",
    "rotation_quaternion": "QUATERNION",
    "rotation_axis_angle": "AXIS_ANGLE",
}


# --- Blender sürüm uyumluluğu ---------------------------------------------
# Blender 4.4+ "slotted action" yapısında Action.fcurves kaldırıldı; eğriler
# artık layer > strip > channelbag altında tutuluyor. Aşağıdaki yardımcılar
# hem eski (Action.fcurves) hem de yeni yapıda çalışır.

def _bone_data_path(bone_name, data_path):
    return 'pose.bones["%s"].%s' % (bpy.utils.escape_identifier(bone_name), data_path)


def _fcurves_find(fcurves, data_path, index):
    find = getattr(fcurves, "find", None)
    if find is not None:
        return find(data_path, index=index)
    for fc in fcurves:
        if fc.data_path == data_path and fc.array_index == index:
            return fc
    return None


def _iter_action_channelbags(action, slot=None):
    for layer in getattr(action, "layers", ()):
        for strip in layer.strips:
            channelbags = getattr(strip, "channelbags", None)
            if not channelbags:
                continue
            for cbag in channelbags:
                if slot is not None and getattr(cbag, "slot_handle", None) != slot.handle:
                    continue
                yield cbag


def iter_action_fcurves(action, slot=None):
    """Action içindeki tüm fcurve'leri sürümden bağımsız olarak dolaşır."""
    fcurves = getattr(action, "fcurves", None)
    if fcurves is not None:
        for fc in fcurves:
            yield fc
        return
    for cbag in _iter_action_channelbags(action, slot):
        for fc in cbag.fcurves:
            yield fc


def find_action_fcurve(action, data_path, index, slot=None):
    """Verilen data_path/index eğrisini sürümden bağımsız olarak bulur."""
    fcurves = getattr(action, "fcurves", None)
    if fcurves is not None:
        return _fcurves_find(fcurves, data_path, index)
    for cbag in _iter_action_channelbags(action, slot):
        fc = _fcurves_find(cbag.fcurves, data_path, index)
        if fc:
            return fc
    return None


def get_action_slot(obj):
    adt = obj.animation_data
    return getattr(adt, "action_slot", None) if adt else None


def _timeline_frame(t_sec, fps):
    """Saniyeyi kareye çevirir; yuvarlama kaymasını tam kareye oturtur."""
    frame = t_sec * fps
    nearest = round(frame)
    if abs(frame - nearest) < 0.01:
        return float(nearest)
    return round(frame, 4)


# --- Timeline verisi çıkarma / uygulama -----------------------------------

def extract_bone_timeline_data(action, bone_name, start_frame, fps, slot=None):
    """Bake edilmemiş, Action'daki orijinal fcurve/keyframe verisini kemik bazında çıkarır."""
    data = {}
    for data_path, axis_count in _TIMELINE_CHANNELS:
        full_path = _bone_data_path(bone_name, data_path)
        channels = {}
        for axis in range(axis_count):
            fcurve = find_action_fcurve(action, full_path, axis, slot)
            if not fcurve or not fcurve.keyframe_points:
                continue
            keys = []
            for kp in fcurve.keyframe_points:
                keys.append({
                    "t": round((kp.co.x - start_frame) / fps, 6),
                    "v": round(kp.co.y, 6),
                    "interp": kp.interpolation,
                    "easing": kp.easing,
                    "hl": [round((kp.handle_left.x - start_frame) / fps, 6),
                           round(kp.handle_left.y, 6)],
                    "hr": [round((kp.handle_right.x - start_frame) / fps, 6),
                           round(kp.handle_right.y, 6)],
                    "hlt": kp.handle_left_type,
                    "hrt": kp.handle_right_type,
                })
            if keys:
                channels[str(axis)] = keys
        if channels:
            data[data_path] = channels
    return data


def apply_bone_timeline_data(pb, action, curve_data, fps):
    """extract_bone_timeline_data ile çıkarılan veriyi Action'a birebir uygular."""
    for data_path, axes in curve_data.items():
        prop = getattr(pb, data_path, None)
        if prop is None:
            continue

        for axis_str, keys in axes.items():
            axis = int(axis_str)
            if axis >= len(prop):
                continue

            keys_by_frame = {}
            for k in keys:
                frame = _timeline_frame(k.get("t", 0.0), fps)
                prop[axis] = k.get("v", 0.0)
                pb.keyframe_insert(data_path=data_path, index=axis, frame=frame)
                keys_by_frame[round(frame, 4)] = k

            fcurve = find_action_fcurve(
                action, _bone_data_path(pb.name, data_path), axis, get_action_slot(pb.id_data)
            )
            if not fcurve:
                continue

            for kp in fcurve.keyframe_points:
                k = keys_by_frame.get(round(kp.co.x, 4))
                if k is None:
                    continue
                try:
                    kp.interpolation = k.get("interp", "BEZIER")
                    kp.easing = k.get("easing", "AUTO")
                    kp.handle_left_type = k.get("hlt", "FREE")
                    kp.handle_right_type = k.get("hrt", "FREE")
                except TypeError:
                    pass
                hl = k.get("hl")
                hr = k.get("hr")
                if hl:
                    kp.handle_left = (_timeline_frame(hl[0], fps), hl[1])
                if hr:
                    kp.handle_right = (_timeline_frame(hr[0], fps), hr[1])
            fcurve.update()


def _normalize_bb_channel(value, axis_count=3):
    """Blockbench animation format fixer for (position/rotation/scale):
      1) {"0.0417": [x,y,z], ...}  -> normal keyframe example
      2) [x, y, z]                 -> static key without t=0
      3) 0 / 1 like single number   -> spetialy for scale, one variable for 3 axis (etc. "scale": 0)
    This method turned this like {t_str: [x,y,z]} """

    if isinstance(value, dict):
        for t_str, kv in value.items():
            if isinstance(kv, dict):
                raise ValueError(
                    f"A Bezier/curved keyframe was detected (t={t_str}). "
                    "This plugin only supports Step/Linear interpolation. "
                    "Please use ‘Bake Animation’ in Blockbench to "
                    "convert this animation to straight keyframes and export it again."
                )
        return value
    
    if isinstance(value, dict):
        return value
    if isinstance(value, (int, float)):
        return {"0.0": [value] * axis_count}
    if isinstance(value, (list, tuple)):
        return {"0.0": list(value)}
    return {}

def apply_bone_timeline_rotation_mode(pb, curve_data):
    for data_path, mode in _TIMELINE_ROTATION_MODE.items():
        if data_path in curve_data:
            pb.rotation_mode = mode
            return


# ==========================================
# 2c. EXPORT META (exportMeta.json) SENKRONİZASYON SİSTEMİ
# ==========================================
_HYP_EXPORT_META_FILENAME = "exportMeta.json"
_HYP_UNLISTED_GROUP = "Unlisted"


def _hyp_export_meta_path():
    """.blend dosyasının yanındaki exportMeta.json yolunu döndürür.
    Blend dosyası henüz kaydedilmemişse (göreli '//' yolu çözülemez) None döner."""
    if not bpy.data.filepath:
        return None
    return bpy.path.abspath("//" + _HYP_EXPORT_META_FILENAME)


def _hyp_load_export_meta():
    """exportMeta.json'ı okur; yoksa varsayılan şablonla oluşturur."""
    default = {"version": "1.0", "groups": {}}
    path = _hyp_export_meta_path()
    if not path:
        return default
    if not os.path.isfile(path):
        _hyp_save_export_meta(default)
        return default
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except (json.JSONDecodeError, OSError):
        data = default
    data.setdefault("version", "1.0")
    data.setdefault("groups", {})
    return data


def _hyp_save_export_meta(data):
    """exportMeta.json'ı diske yazar. Blend dosyası kaydedilmemişse sessizce atlar."""
    path = _hyp_export_meta_path()
    if not path:
        return
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=4, ensure_ascii=False)


def _hyp_sync_export_meta():
    """Sahnedeki action'lar ile meta dosyasını senkronize eder:
    - Meta'da kayıtlı ama sahnede artık olmayan action'ları meta'dan temizler.
    - Boş kalan grupları siler.
    Dönüş: (meta_dict, action_name -> group_name eşleme sözlüğü)"""
    meta = _hyp_load_export_meta()
    groups = meta.setdefault("groups", {})

    existing_action_names = {a.name for a in bpy.data.actions}
    changed = False
    for g_name in list(groups.keys()):
        cleaned = [n for n in groups[g_name] if n in existing_action_names]
        if len(cleaned) != len(groups[g_name]):
            changed = True
        if cleaned:
            groups[g_name] = cleaned
        else:
            del groups[g_name]
            changed = True

    if changed:
        _hyp_save_export_meta(meta)

    action_to_group = {}
    for g_name, action_names in groups.items():
        for a_name in action_names:
            action_to_group[a_name] = g_name

    return meta, action_to_group


def _hyp_move_action_to_group(groups, action_name, target_group):
    """Bir action'ı, kayıtlı olduğu her gruptan çıkarıp hedef gruba taşır.
    Kaynak grup taşıma sonrası boş kalırsa meta'dan tamamen silinir."""
    for g_name in list(groups.keys()):
        if g_name == target_group:
            continue
        if action_name in groups[g_name]:
            groups[g_name].remove(action_name)
            if not groups[g_name]:
                del groups[g_name]

    groups.setdefault(target_group, [])
    if action_name not in groups[target_group]:
        groups[target_group].append(action_name)


def _hyp_group_name_from_filepath(filepath):
    """Export edilen dosyanın adından (uzantısız) otomatik grup adı türetir.
    Örn: '.../sword.animation.json' -> 'sword.animation'
    Windows/Unix ayraçlarının ikisini de (os.path'in hangi platformda
    çalıştığından bağımsız olarak) doğru şekilde işler."""
    normalized = (filepath or "").replace("\\", "/").rstrip("/")
    base = normalized.rsplit("/", 1)[-1]
    name, _ext = os.path.splitext(base)
    return name.strip()


# ==========================================
# 3. EXPORT & IMPORT
# ==========================================
class HYP_OT_export_animations(bpy.types.Operator, ExportHelper):
    bl_idname = "hyp.export_animations"
    bl_label = "Export Blockbench JSON"
    filename_ext = ".json"
    filter_glob: bpy.props.StringProperty(default="*.json", options={'HIDDEN'})

    export_metadata_timeline: bpy.props.BoolProperty(
        name="Export Action Timeline MetaData",
        description=(
            "It's added unbaked original keyframe (timeline) value inside MetaData. "
            "If this method is activated, system use firstly original timeline than baked timeline."
        ),
        default=False,
    )

    optimize_export: bpy.props.BoolProperty(
        name="Optimized Export (Simplify Curves)",
        description=(
            "clean_keyframes removes only completely static/flat sections. When this option is "
            "enabled, unnecessary intermediate keys in areas with smooth/curved motion "
            "generated by the RDP algorithm are also simplified within the tolerance limit. "
            "Start and end keys are always preserved. When disabled, the export "
            "behavior remains exactly the same as before."
        ),
        default=False,
    )
    optimize_tolerance: bpy.props.FloatProperty(
        name="Simplify Tolerance",
        description=(
            "RDP smoothing tolerance. The higher it is, the less detail remains. "
            "(The file size decreases) but the risk of deviation from the curve also increases. It is recommended to "
            "start with low values and increase them while testing the output."
        ),
        default=0.05,
        min=0.0001,
        max=5.0,
        precision=4,
    )

    def invoke(self, context, event):
        meta, action_to_group = _hyp_sync_export_meta()

        if not bpy.data.filepath:
            self.report({'WARNING'}, "This blend fie doesn't saved: exportMeta.json does not created.")

        context.scene.hyp_export_list.clear()
        context.scene.hyp_export_groups.clear()

        # Grup sırası: meta'daki gruplar (alfabetik) + en sonda sanal "Unlisted"
        group_names_ordered = sorted(meta.get("groups", {}).keys())
        has_unlisted = any(
            action.name not in action_to_group for action in bpy.data.actions
        )
        if has_unlisted:
            group_names_ordered.append(_HYP_UNLISTED_GROUP)

        for action in bpy.data.actions:
            item = context.scene.hyp_export_list.add()
            item.action_name = action.name
            item.group_name = action_to_group.get(action.name, _HYP_UNLISTED_GROUP)
            item.export = False
            if (context.active_object
                    and context.active_object.animation_data
                    and context.active_object.animation_data.action == action):
                item.export = True

        for g_name in group_names_ordered:
            g_item = context.scene.hyp_export_groups.add()
            g_item.group_name = g_name
            g_item.select_all = False

        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "export_metadata_timeline")
        layout.separator()

        opt_box = layout.box()
        opt_box.prop(self, "optimize_export")
        if self.optimize_export:
            opt_box.prop(self, "optimize_tolerance")
        layout.separator()

        meta_box = layout.box()
        meta_box.label(text="Export Meta - Point Group", icon='FILE_FOLDER')
        preview_group = _hyp_group_name_from_filepath(self.filepath)
        if preview_group:
            meta_box.label(text="File name based group: \"%s\"" % preview_group, icon='GROUP')
        else:
            meta_box.label(text="When file name selected, group automatically created.", icon='INFO')

        layout.separator()
        box = layout.box()
        box.label(text="Select Actions to Export:", icon='ACTION')

        for g_item in context.scene.hyp_export_groups:
            g_box = box.box()
            header = g_box.row(align=True)
            header.prop(g_item, "select_all", text="")
            icon = 'GHOST_ENABLED' if g_item.group_name == _HYP_UNLISTED_GROUP else 'GROUP'
            header.label(text=g_item.group_name, icon=icon)

            for item in context.scene.hyp_export_list:
                if item.group_name == g_item.group_name:
                    g_box.prop(item, "export", text=item.action_name)

    def execute(self, context):
        rig = context.active_object
        
        if not rig or rig.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an Armature (Rig)!")
            return {'CANCELLED'}
            
        fps = context.scene.render.fps / context.scene.render.fps_base
        
        export_dict = {"format_version": "1.8.0", "animations": {}}
        original_action = rig.animation_data.action if rig.animation_data else None
        exported_action_names = []

        current_frame = context.scene.frame_current

        
        pose_bones = list(rig.pose.bones)
        rest_local_inv = {}
        for pb in pose_bones:
            if pb.parent:
                rest_local = pb.bone.parent.matrix_local.inverted() @ pb.bone.matrix_local
            else:
                rest_local = pb.bone.matrix_local
            rest_local_inv[pb.name] = rest_local.inverted()

        for item in context.scene.hyp_export_list:
            if not item.export: continue
            action = bpy.data.actions.get(item.action_name)
            if not action: continue
            
            if not rig.animation_data: rig.animation_data_create()
            rig.animation_data.action = action
            
            start_f = int(action.frame_range[0])
            end_f = int(action.frame_range[1])
            anim_length = round((end_f - start_f) / fps, 4)
            
            # --- MARKER PARSING ---
            loop_mode = "play_once"
            sound_effects = {}
            timeline = {}
            
            all_markers = []
            if action.pose_markers:
                all_markers.extend(action.pose_markers)
                
            for m in context.scene.timeline_markers:
                if start_f <= m.frame <= end_f:
                    if not any(pm.frame == m.frame and pm.name == m.name for pm in action.pose_markers):
                        all_markers.append(m)

            all_markers.sort(key=lambda mk: mk.frame)

            for m in all_markers:
                t_sec = round((m.frame - start_f) / fps, 4)
                if t_sec < 0: t_sec = 0.0
                t_str = str(t_sec)
                
                m_name = m.name

                # L:/S:/T:
                remaining = m_name

                lm = re.match(r'\s*L:\s*(?:"([^"]+)"|(\S+))\s*', remaining, re.IGNORECASE)
                if lm:
                    loop_mode = lm.group(1) or lm.group(2)
                    remaining = remaining[lm.end():]

                sm = re.match(r'\s*S:\s*(?:"([^"]+)"|(\S+))\s*', remaining, re.IGNORECASE)
                if sm:
                    s_effect = sm.group(1) or sm.group(2)
                    sound_effects[t_str] = {"effect": s_effect}
                    remaining = remaining[sm.end():]

                tm = re.match(r'\s*T:\s*(?:"([^"]+)"|(\S+))\s*', remaining, re.IGNORECASE)
                if tm:
                    explicit_tl = tm.group(1) or tm.group(2)
                    remaining = remaining[tm.end():]
                else:
                    explicit_tl = ""

                # (Implicit Timeline)
                implicit_tl = remaining.strip()
                
                final_tl = explicit_tl
                if implicit_tl:
                    if final_tl and not final_tl.endswith(";"):
                        final_tl += ";"
                    final_tl += implicit_tl
                    
                if final_tl:
                    if t_str in timeline:
                        if not timeline[t_str].endswith(";"):
                            timeline[t_str] += ";"
                        timeline[t_str] += final_tl
                    else:
                        timeline[t_str] = final_tl

            # --- BONES PARSING ---
            bones_data = {}
            prev_eulers = {}

            for pb in pose_bones:
                bones_data[pb.name] = {"rotation": {}, "position": {}, "scale": {}}

            for frame in range(start_f, end_f + 1):
                context.scene.frame_set(frame)
                b_t_str = str(round((frame - start_f) / fps, 4))

                for pb in pose_bones:
                    if pb.parent:
                        local_matrix = pb.parent.matrix.inverted() @ pb.matrix
                    else:
                        local_matrix = pb.matrix

                    offset_mat = rest_local_inv[pb.name] @ local_matrix

                    loc_bl, rot_quat_bl, scale_bl = offset_mat.decompose()
                    clean_offset_mat = mathutils.Matrix.LocRotScale(loc_bl, rot_quat_bl, scale_bl)
                     
                    
                    if pb.name == "itemgrip_right" or pb.name == "itemgrip_left":
                        #print(frame, pb.name, "Loc:", loc_bl.x, loc_bl.y, loc_bl.z)
                        if pb.name in prev_eulers:
                            euler_bl = clean_offset_mat.to_euler('YZX', prev_eulers[pb.name])
                        else:
                            euler_bl = clean_offset_mat.to_euler('YZX')

                        prev_eulers[pb.name] = euler_bl.copy()

                        rx = round(math.degrees(euler_bl.x * -1.0), 5)
                        ry = round(math.degrees(euler_bl.y * -1.0), 5)
                        rz = round(math.degrees(euler_bl.z * -1.0), 5)
                    else:
                        if pb.name in prev_eulers:
                            euler_bl = clean_offset_mat.to_euler('XYZ', prev_eulers[pb.name])
                        else:
                            euler_bl = clean_offset_mat.to_euler('XYZ')
                            
                        prev_eulers[pb.name] = euler_bl.copy()
                        
                        rx = round(math.degrees(euler_bl.x * -1.0), 2)
                        ry = round(math.degrees(euler_bl.z), 2)
                        rz = round(math.degrees(euler_bl.y), 2)

                    rx = 0.0 if rx == 0.0 else rx
                    ry = 0.0 if ry == 0.0 else ry
                    rz = 0.0 if rz == 0.0 else rz
                    
                    bones_data[pb.name]["rotation"][b_t_str] = [rx, ry, rz]
                    
                    if pb.name == "itemgrip_right" or pb.name == "itemgrip_left":
                        px = round(loc_bl.x * -16.0, 3)
                        py = round(loc_bl.y * 16.0, 3)
                        pz = round(loc_bl.z * 16.0, 3)
                    else:
                        px = round(loc_bl.x * -16.0, 3)
                        py = round(loc_bl.z * -16.0, 3)
                        pz = round(loc_bl.y * 16.0, 3)

                    px = 0.0 if px == 0.0 else px
                    py = 0.0 if py == 0.0 else py
                    pz = 0.0 if pz == 0.0 else pz

                    bones_data[pb.name]["position"][b_t_str] = [px, py, pz]

                    if pb.name == "itemgrip_right" or pb.name == "itemgrip_left":
                        sx = round(scale_bl.x, 5)
                        sy = round(scale_bl.y, 5)
                        sz = round(scale_bl.z, 5)
                    else:
                        sx = round(scale_bl.x, 5)
                        sy = round(scale_bl.z, 5)
                        sz = round(scale_bl.y, 5)

                    sx = 0.0 if sx == 0.0 else sx
                    sy = 0.0 if sy == 0.0 else sy
                    sz = 0.0 if sz == 0.0 else sz

                    bones_data[pb.name]["scale"][b_t_str] = [sx, sy, sz]

            for b_name in list(bones_data.keys()):
                pos_dict = clean_keyframes(bones_data[b_name]["position"])
                rot_dict = clean_keyframes(bones_data[b_name]["rotation"])
                scale_dict = clean_keyframes(bones_data[b_name]["scale"])

                if self.optimize_export:
                    pos_dict = simplify_keyframes(pos_dict, self.optimize_tolerance)
                    rot_dict = simplify_keyframes(rot_dict, self.optimize_tolerance)
                    scale_dict = simplify_keyframes(scale_dict, self.optimize_tolerance)

                pos_total_movement = sum(sum(abs(v) for v in val) for val in pos_dict.values())
                rot_total_movement = sum(sum(abs(v) for v in val) for val in rot_dict.values())
                # Scale için referans 1.0'dır (identity); 1.0'dan sapma "hareket" sayılır
                scale_total_deviation = sum(sum(abs(v - 1.0) for v in val) for val in scale_dict.values())
                
                if pos_total_movement < 0.001 and rot_total_movement < 0.001 and scale_total_deviation < 0.001:
                    del bones_data[b_name]
                else:
                    bones_data[b_name]["position"] = pos_dict
                    bones_data[b_name]["rotation"] = rot_dict
                    bones_data[b_name]["scale"] = scale_dict

            # --- METADATA TIMELINE (RAW, NON-BAKED) ---
            metadata_timeline = {}
            if self.export_metadata_timeline:
                action_slot = get_action_slot(rig)
                for b_name in bones_data.keys():
                    bone_curve_data = extract_bone_timeline_data(
                        action, b_name, start_f, fps, action_slot
                    )
                    if bone_curve_data:
                        metadata_timeline[b_name] = bone_curve_data

            # --- ASSEMBLE ACTION DICT ---
            action_export_data = {
                "loop": loop_mode,
                "animation_length": anim_length,
                "bones": bones_data
            }

            if sound_effects:
                action_export_data["sound_effects"] = sound_effects
            if timeline:
                action_export_data["timeline"] = timeline

            action_export_data["hyp_metadata"] = {"fps": round(fps, 2)}

            if metadata_timeline:
                action_export_data["hyp_metadata"]["action_timeline"] = metadata_timeline

            export_dict["animations"][action.name] = action_export_data
            exported_action_names.append(action.name)

        if original_action: rig.animation_data.action = original_action
        
        context.scene.frame_set(current_frame)

        json_str = json.dumps(export_dict, indent=4)
        
        compact_json_str = re.sub(
            r'\[\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*,\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*,\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*\]',
            r'[\1, \2, \3]',
            json_str
        )

        with open(self.filepath, 'w', encoding='utf-8') as f:
            f.write(compact_json_str)

        # --- EXPORT META GÜNCELLEME (grup taşıma / migration) ---
        # Grup adı, kullanıcının verdiği çıktı dosya adından otomatik türetilir
        # (örn. "sword.animation.json" -> grup: "sword.animation").
        target_group = _hyp_group_name_from_filepath(self.filepath)
        if target_group and exported_action_names:
            meta = _hyp_load_export_meta()
            groups = meta.setdefault("groups", {})
            for a_name in exported_action_names:
                _hyp_move_action_to_group(groups, a_name, target_group)
            meta["groups"] = groups
            _hyp_save_export_meta(meta)

        self.report({'INFO'}, "Punchy JSON Exported Successfully!")
        return {'FINISHED'}


class HYP_OT_import_animations(bpy.types.Operator, ImportHelper):
    bl_idname = "hyp.import_animations"
    bl_label = "Import Blockbench JSON"
    filename_ext = ".json"
    filter_glob: bpy.props.StringProperty(default="*.json", options={'HIDDEN'})

    def execute(self, context):
        rig = context.active_object

        if not rig or rig.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select a Armature (Rig)!")
            return {'CANCELLED'}

        with open(self.filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        animations = data.get("animations", {})
        
        for anim_name, anim_data in animations.items():
            new_name = anim_name
            if new_name in bpy.data.actions:
                new_name += "_copy"
            action = bpy.data.actions.new(name=new_name)
            if not rig.animation_data:
                rig.animation_data_create()
            rig.animation_data.action = action
            
            fps = anim_data.get("hyp_metadata", {}).get("fps", context.scene.render.fps)

            # --- MARKERS IMPORT ---
            markers_by_frame = {}

            # 1. Loop Mode
            loop_mode = anim_data.get("loop", "play_once")
            if loop_mode != "play_once":
                markers_by_frame[0] = [f'L:"{loop_mode}"']

            # 2. Sound Effects
            sound_effects = anim_data.get("sound_effects", {})
            for t_str, s_data in sound_effects.items():
                frame = int(round(float(t_str) * fps))
                effect_name = s_data.get("effect", "")
                if effect_name:
                    if frame not in markers_by_frame:
                        markers_by_frame[frame] = []
                    markers_by_frame[frame].append(f'S:"{effect_name}"')

            # 3. Timeline Effects
            timeline = anim_data.get("timeline", {})
            for t_str, t_data in timeline.items():
                frame = int(round(float(t_str) * fps))
                if frame not in markers_by_frame:
                    markers_by_frame[frame] = []
                
                if isinstance(t_data, list):
                    flags = t_data
                else:
                    flags = [x.strip() for x in str(t_data).split(';') if x.strip()]
                    
                markers_by_frame[frame].extend(flags)

            # Character limit (40/60) marker creator (Frame Floating)
            for frame, parts in markers_by_frame.items():
                chunks = []
                current_str = ""
                for part in parts:
                    is_timeline = not (part.startswith("L:") or part.startswith("S:"))
                    part_str = part + ";" if is_timeline else part + " "

                    limit = 40 if ("L:" in current_str or "S:" in current_str) else 60

                    if len(current_str) + len(part_str) > limit and current_str:
                        chunks.append(current_str)
                        current_str = ""

                    current_str += part_str

                if current_str:
                    chunks.append(current_str)

                if frame == 0 and len(chunks) > 1:
                    n = len(chunks)
                    for i, chunk in enumerate(chunks):
                        offset = -(n - 1 - i)
                        m = action.pose_markers.new(name=chunk.strip())
                        m.frame = offset
                else:
                    current_frame = frame
                    for chunk in chunks:
                        m = action.pose_markers.new(name=chunk.strip())
                        m.frame = current_frame
                        current_frame += 1

            # --- BONES IMPORT ---
            bones_data = anim_data.get("bones", {})
            metadata_timeline = anim_data.get("hyp_metadata", {}).get("action_timeline", {})
            timeline_fcurve_paths = set()

            for b_name, b_data in bones_data.items():
                pb = rig.pose.bones.get(b_name)
                if not pb: continue

                bone_curve_data = metadata_timeline.get(b_name)
                if bone_curve_data:
                    apply_bone_timeline_rotation_mode(pb, bone_curve_data)
                    apply_bone_timeline_data(pb, action, bone_curve_data, fps)
                    for data_path in bone_curve_data.keys():
                        timeline_fcurve_paths.add(_bone_data_path(b_name, data_path))
                    continue

                pb.rotation_mode = 'XYZ'
                
                try:
                    for t_str, pos in _normalize_bb_channel(b_data.get("position", {})).items():
                        frame = int(round(float(t_str) * fps))

                        if pb.name == "itemgrip_right" or pb.name == "itemgrip_left":
                            pb.location = (
                                pos[0] / -16.0,
                                pos[1] /  16.0,
                                pos[2] /  16.0,
                            )
                        else:
                            pb.location = (
                                pos[0] / -16.0,
                                pos[2] /  16.0,
                                pos[1] / -16.0,
                            )
                        pb.keyframe_insert(data_path="location", frame=frame)

                    
                    for t_str, rot in _normalize_bb_channel(b_data.get("rotation", {})).items():
                        frame = int(round(float(t_str) * fps))

                        if pb.name == "itemgrip_right" or pb.name == "itemgrip_left":
                            pb.rotation_euler = [
                                math.radians(rot[0] * -1.0),
                                math.radians(rot[2]),
                                math.radians(rot[1]),
                            ]
                        else:
                            pb.rotation_euler = [
                                math.radians(rot[0] * -1.0),
                                math.radians(rot[2] *  1.0),
                                math.radians(rot[1] *  1.0),
                            ]
                        pb.keyframe_insert(data_path="rotation_euler", frame=frame)

                    
                    for t_str, scl in _normalize_bb_channel(b_data.get("scale", {})).items():
                        frame = int(round(float(t_str) * fps))

                        if pb.name == "itemgrip_right" or pb.name == "itemgrip_left":
                            pb.scale = (scl[0], scl[1], scl[2])
                        else:
                            pb.scale = (scl[0], scl[2], scl[1])
                        pb.keyframe_insert(data_path="scale", frame=frame)
                        
                except ValueError as e:
                    self.report({'ERROR'}, str(e))
                    return {'CANCELLED'}

            for fcurve in iter_action_fcurves(action, get_action_slot(rig)):
                # MetaData timeline'dan geri yüklenen orijinal interpolasyonları bozma
                if fcurve.data_path in timeline_fcurve_paths:
                    continue
                for key in fcurve.keyframe_points:
                    key.interpolation = 'LINEAR'

        self.report({'INFO'}, "Blockbench JSON Imported!")
        return {'FINISHED'}


# ==========================================
# 4. UI PANEL
# ==========================================
class VIEW3D_PT_hyp_anim_panel(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Punchy Exporer'
    bl_label = 'Animation System'

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def draw(self, context):
        layout = self.layout

        row = layout.row(align=True)
        row.operator("hyp.import_animations", text="Import JSON", icon='IMPORT')
        row.operator("hyp.export_animations", text="Export JSON", icon='EXPORT')
        layout.separator()
        
        link_col = layout.column(align=True)
        op1 = link_col.operator("wm.url_open", text="Punchy Wiki", icon='HELP')
        op1.url = "https://github.com/punchy-guys/punchy-wiki/wiki"
        op2 = link_col.operator("wm.url_open", text="Hyp's Punchy Blender Wiki", icon='URL')
        op2.url = "https://github.com/Hamza-inc/Punchy-Blender/wiki"


# ==========================================
# 5. REGISTRATION
# ==========================================
classes = (
    HypActionExportItem,
    HypExportGroupItem,
    HYP_OT_export_animations,
    HYP_OT_import_animations,
    VIEW3D_PT_hyp_anim_panel,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.hyp_export_list = bpy.props.CollectionProperty(type=HypActionExportItem)
    bpy.types.Scene.hyp_export_groups = bpy.props.CollectionProperty(type=HypExportGroupItem)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.hyp_export_list
    del bpy.types.Scene.hyp_export_groups

if __name__ == "__main__":
    register()