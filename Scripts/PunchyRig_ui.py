import bpy
import json
import math
import mathutils
import re
from bpy_extras.io_utils import ExportHelper, ImportHelper


# ==========================================
# 1. PROPERTY GROUPS
# ==========================================
class HypActionExportItem(bpy.types.PropertyGroup):
    action_name: bpy.props.StringProperty()
    export: bpy.props.BoolProperty(default=True)


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

            # 1) Keyframe'leri oluştur. keyframe_insert her Blender sürümünde
            #    gerekli slot/layer/channelbag yapısını kendisi kurar.
            keys_by_frame = {}
            for k in keys:
                frame = _timeline_frame(k.get("t", 0.0), fps)
                prop[axis] = k.get("v", 0.0)
                pb.keyframe_insert(data_path=data_path, index=axis, frame=frame)
                keys_by_frame[round(frame, 4)] = k

            # 2) Orijinal interpolasyon ve handle bilgilerini geri yükle
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


def apply_bone_timeline_rotation_mode(pb, curve_data):
    for data_path, mode in _TIMELINE_ROTATION_MODE.items():
        if data_path in curve_data:
            pb.rotation_mode = mode
            return


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

    def invoke(self, context, event):
        context.scene.hyp_export_list.clear()
        for action in bpy.data.actions:
            item = context.scene.hyp_export_list.add()
            item.action_name = action.name
            item.export = False
            if (context.active_object
                    and context.active_object.animation_data
                    and context.active_object.animation_data.action == action):
                item.export = True
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "export_metadata_timeline")

        box = layout.box()
        box.label(text="Select Actions to Export:", icon='ACTION')
        for item in context.scene.hyp_export_list:
            box.prop(item, "export", text=item.action_name)

    def execute(self, context):
        rig = context.active_object
        
        if not rig or rig.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an Armature (Rig)!")
            return {'CANCELLED'}
            
        fps = context.scene.render.fps / context.scene.render.fps_base
        
        export_dict = {"format_version": "1.8.0", "animations": {}}
        original_action = rig.animation_data.action if rig.animation_data else None

        current_frame = context.scene.frame_current

        # Rest-pose (bind) matrisleri karadan kareye değişmez; tüm export boyunca bir kez hesaplanır
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
                    # Pose marker'da zaten aynı karede aynı isimde marker varsa çakışmayı önle
                    if not any(pm.frame == m.frame and pm.name == m.name for pm in action.pose_markers):
                        all_markers.append(m)

            for m in all_markers:
                t_sec = round((m.frame - start_f) / fps, 4)
                if t_sec < 0: t_sec = 0.0
                t_str = str(t_sec)
                
                m_name = m.name
                
                # Loop değerini çek (Tırnaklı veya tırnaksız)
                lm = re.search(r'L:\s*(?:"([^"]+)"|(\S+))', m_name, re.IGNORECASE)
                if lm: 
                    loop_mode = lm.group(1) or lm.group(2)
                
                # Sound değerini çek (Tırnaklı veya tırnaksız)
                sm = re.search(r'S:\s*(?:"([^"]+)"|(\S+))', m_name, re.IGNORECASE)
                if sm: 
                    s_effect = sm.group(1) or sm.group(2)
                    sound_effects[t_str] = {"effect": s_effect}
                    
                # Timeline değerini belirteçle çekildiyse (Timeline:"...") al
                tm = re.search(r'T:\s*(?:"([^"]+)"|(\S+))', m_name, re.IGNORECASE)
                explicit_tl = tm.group(1) or tm.group(2) if tm else ""
                
                # Geriye kalan ve hiçbir ön eki olmayan metni (Implicit Timeline) bul
                rest = m_name
                rest = re.sub(r'L:\s*(?:"[^"]+"|\S+)', '', rest, flags=re.IGNORECASE)
                rest = re.sub(r'S:\s*(?:"[^"]+"|\S+)', '', rest, flags=re.IGNORECASE)
                rest = re.sub(r'T:\s*(?:"[^"]+"|\S+)', '', rest, flags=re.IGNORECASE)
                
                implicit_tl = rest.strip()
                
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
                bones_data[pb.name] = {"rotation": {}, "position": {}}

            for frame in range(start_f, end_f + 1):
                context.scene.frame_set(frame)
                b_t_str = str(round((frame - start_f) / fps, 4))

                for pb in pose_bones:
                    if pb.parent:
                        local_matrix = pb.parent.matrix.inverted() @ pb.matrix
                    else:
                        local_matrix = pb.matrix

                    offset_mat = rest_local_inv[pb.name] @ local_matrix

                    loc_bl, rot_quat_bl, _ = offset_mat.decompose()
                    clean_offset_mat = mathutils.Matrix.LocRotScale(loc_bl, rot_quat_bl, None)
                    
                    if pb.name == "itemgrip_right":
                        temp_euler = clean_offset_mat.to_euler('XYZ')
                        inverted_euler = mathutils.Euler((-temp_euler.x, -temp_euler.y, -temp_euler.z), 'XYZ')
                        final_mat = inverted_euler.to_matrix().to_4x4()
                        
                        if pb.name in prev_eulers:
                            euler_bl = final_mat.to_euler('XYZ', prev_eulers[pb.name])
                        else:
                            euler_bl = final_mat.to_euler('XYZ')
                            
                        prev_eulers[pb.name] = euler_bl.copy()
                        
                        rx = round(math.degrees(euler_bl.x), 2)
                        ry = round(math.degrees(euler_bl.y - euler_bl.z), 2)
                        rz = round(math.degrees(euler_bl.z - euler_bl.y), 2)
                        
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
                    
                    if pb.name == "itemgrip_right":
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

            for b_name in list(bones_data.keys()):
                pos_dict = clean_keyframes(bones_data[b_name]["position"])
                rot_dict = clean_keyframes(bones_data[b_name]["rotation"])
                
                pos_total_movement = sum(sum(abs(v) for v in val) for val in pos_dict.values())
                rot_total_movement = sum(sum(abs(v) for v in val) for val in rot_dict.values())
                
                if pos_total_movement < 0.001 and rot_total_movement < 0.001:
                    del bones_data[b_name]
                else:
                    bones_data[b_name]["position"] = pos_dict
                    bones_data[b_name]["rotation"] = rot_dict

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
            self.report({'ERROR'}, "Lütfen bir Armature (Rig) seçin!")
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
                
                # Veriyi noktalı virgülden bölerek listeye ekle
                if isinstance(t_data, list):
                    flags = t_data
                else:
                    flags = [x.strip() for x in str(t_data).split(';') if x.strip()]
                    
                markers_by_frame[frame].extend(flags)

            # Karakter Limitini (40/60) Yöneterek Markerları Oluştur (Frame Kaydırma)
            for frame, parts in markers_by_frame.items():
                current_str = ""
                current_frame = frame  # Başlangıç karesi
                
                for part in parts:
                    # Loop veya Sound değilse timeline flag'idir, sonuna noktalı virgül koy
                    is_timeline = not (part.startswith("L:") or part.startswith("S:"))
                    part_str = part + ";" if is_timeline else part + " "
                    
                    # İçerikte Loop veya Sound varsa limit 40, sadece timeline ise limit 60
                    limit = 40 if ("L:" in current_str or "S:" in current_str) else 60
                    
                    # Eğer bu parçayı eklemek limiti aşıyorsa, mevcut stringi marker yap ve sonraki kareye geç
                    if len(current_str) + len(part_str) > limit and current_str:
                        m = action.pose_markers.new(name=current_str.strip())
                        m.frame = current_frame
                        current_str = ""
                        current_frame += 1  # Limiti aştığı için bir sonraki kareye kaydır!
                        
                    current_str += part_str
                    
                # Kalan metni son marker olarak ekle
                if current_str:
                    m = action.pose_markers.new(name=current_str.strip())
                    m.frame = current_frame

            # --- BONES IMPORT ---
            bones_data = anim_data.get("bones", {})
            metadata_timeline = anim_data.get("hyp_metadata", {}).get("action_timeline", {})
            timeline_fcurve_paths = set()

            for b_name, b_data in bones_data.items():
                pb = rig.pose.bones.get(b_name)
                if not pb: continue

                # MetaData içinde bake edilmemiş timeline verisi varsa, onu öncelikli kullan
                bone_curve_data = metadata_timeline.get(b_name)
                if bone_curve_data:
                    apply_bone_timeline_rotation_mode(pb, bone_curve_data)
                    apply_bone_timeline_data(pb, action, bone_curve_data, fps)
                    for data_path in bone_curve_data.keys():
                        timeline_fcurve_paths.add(_bone_data_path(b_name, data_path))
                    continue

                pb.rotation_mode = 'XYZ'

                # Pozisyon Aktarımı
                for t_str, pos in b_data.get("position", {}).items():
                    frame = int(round(float(t_str) * fps))
                    
                    if pb.name == "itemgrip_right":
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
                    
                # Rotasyon Aktarımı
                for t_str, rot in b_data.get("rotation", {}).items():
                    frame = int(round(float(t_str) * fps))
                    
                    if pb.name == "itemgrip_right":
                        pb.rotation_euler = [
                            math.radians(rot[0]),
                            math.radians(rot[1]),
                            math.radians(rot[2]),
                        ]
                    else:
                        pb.rotation_euler = [
                            math.radians(rot[0] * -1.0),
                            math.radians(rot[2] *  1.0),
                            math.radians(rot[1] *  1.0),
                        ]
                    pb.keyframe_insert(data_path="rotation_euler", frame=frame)

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
    HYP_OT_export_animations,
    HYP_OT_import_animations,
    VIEW3D_PT_hyp_anim_panel,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.hyp_export_list = bpy.props.CollectionProperty(type=HypActionExportItem)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.hyp_export_list

if __name__ == "__main__":
    register()