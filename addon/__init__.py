bl_info = {
    "name": "Motion Capture Toolkit",
    "description": "(WIP) Toolkit for working with motion capture data",
    "author": "Ezra Hill",
    "version": (0,0,2),
    "blender" : (3,5,0),
    "category" : "Animation",
}
import bpy
import os
import bmesh
import copy
import mathutils


from bpy.props import StringProperty, BoolProperty, EnumProperty, FloatProperty, IntProperty, FloatVectorProperty, BoolVectorProperty
from random import uniform
from bpy_extras.io_utils import ImportHelper
from bpy.types import Operator
from math import sin
from math import pow



class MotionCaptureSceneProperties(bpy.types.PropertyGroup):
    file_path: StringProperty(name="File Path",
                                        description="Path to BVH Motion Data",
                                        default="",
                                        maxlen=1024,
                                        #subtype="FILE_PATH"
                                        )
    import_as_quaternion: BoolProperty(name="Import as Quaternion",
                                                description="Import as Quaternion",
                                                default=True,
                                                )
    import_scale: FloatProperty(name="Import Scale",
                                            description="Scale of imported motion data",
                                            default=1.0,
                                            min=0.0,
                                        )


class MotionCaptureObjectProperties(bpy.types.PropertyGroup):
    def update_lock_root(self, context):
        # run OT_Apply_Keyframes on update
        bpy.ops.mocaptk.toggle_root_lock()

    def update_start_loop(self, context):
        if self.has_default_start_end is False:
            bpy.ops.mocaptk.set_loop_start_end_from_action()
            self.has_default_start_end = True
    
    def get_bone_names(self, context):
        #tuple of bone names as [(identifier, name, description)]
        bone_names = []
        for armature in [ob for ob in bpy.data.objects if ob.type == 'ARMATURE']:
            for bone in armature.data.bones:
                bone_names.append((bone.name,bone.name,bone.name))

        return tuple(bone_names)

    
    # Getters and setters don't behave appropriately so using Update to initialize Start Loop
    # This is costly and shouldn't be neccesary
    use_loop: BoolProperty(name="Should Loop",
                            description="Should this motion capture data loop?",
                            default=False,
                            update=update_start_loop
                            )
    use_loop_at_start: BoolProperty(name="Loop at Start",
                                    description="Should this motion capture data loop at the start, or on the current frame?",
                                    default=False,
                                    )
    force_loop_offset: IntProperty(name="Force Loop Offset",
                                    description="How many frames to wait before forcing a loop",
                                    default=0,
                                    )
    start_loop: IntProperty(name="Start Loop",
                                        description="First frame to loop motion capture",
                                        min=0,
                                        max=999999,
                                        default=0,
                                        )
    #todo: observer to update when loops change so backups can be updated
    end_loop: IntProperty(name="End",
                            description="Last frame to loop motion capture",
                            min=0,
                            max=999999,
                            default=0
                            )

    # for initializing these to better values
    has_default_start_end: BoolProperty(name="has_default_start_end", default=False)

    is_loc_rot_backed_up: BoolProperty(name="is_loc_rot_backed_up", default=False)

    lock_pos: BoolVectorProperty(name="lock_pos",
                                description="Should location be locked on this axis?",
                                size = 3,
                                default = (False,False,False),
                                update=update_lock_root
                                )
    lock_rot: BoolVectorProperty(name="lock_rot",
                                description="Should rotation be locked on this axis?",
                                size = 4,
                                default = (False,False,False,False),
                                update=update_lock_root
                                )
    clean_threshold: FloatProperty(name="clean_threshold",
                                    description="Threshold for cleaning up keyframes",
                                    default=0.01,
                                    min=0.000000001,
                                    max=1.0
                                    )
    bone_names_enum: EnumProperty(name="Bone Names",
                                    description="All bones in armature",
                                    items=get_bone_names
                                )
    
    # Collection is for all property info that must be arbritrarially sized lists of floats
    # Can store data 1 per frame/keyframe
class MotionCaptureObjectCollections(bpy.types.PropertyGroup):
    root_loc_backup: FloatVectorProperty(name="root_loc_backup",
                                                    description="Default location for root for all frames in range",
                                                    )
    root_rot_backup: FloatVectorProperty(name="root_rot_backup",
                                                    description="Default quaternion rotation for root for all frames in range",
                                                    size = 4
                                                    )

                                    
class MotionCaptureToolkit(bpy.types.Operator):
    bl_idname = "animation.motion_capture_toolkit"
    bl_label = "Motion Capture Toolkit"

    def execute(self,context):
        if context.scene.motion_capture_properties.import_as_quaternion is not True:
            rotate_mode = 'NATIVE'
        else:
            rotate_mode = 'QUATERNION'
        bpy.ops.import_anim.bvh(filepath=context.scene.motion_capture_properties.file_path,rotate_mode=rotate_mode,global_scale=context.scene.motion_capture_properties.import_scale)
        if bpy.context.mode != 'OBJECT':
            print("Not in object mode!")
        return {'FINISHED'}
    
    
    
    
class MotionCaptureToolkitPanel:
    """Motion Capture Toolkit"""
    bl_idname = "object.mocaptool_formatter_panel"
    bl_label = "Motion Capture Toolkit Panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Mocap"
    
    bvh_path: StringProperty(name="BVH Path")

    @classmethod
    def poll(cls, context):
        return context.mode in {'OBJECT','POSE'}
    

class MOCAPTK_PT_MotionCaptureInputFilePanel(MotionCaptureToolkitPanel, bpy.types.Panel):
    bl_idname = "MOCAPTK_PT_input_panel"

    def draw(self,context):
        row = self.layout.row()
        col = self.layout.column()
        scene_mocap_prop = context.scene.motion_capture_properties
        
        row.prop(scene_mocap_prop, "file_path")
        
        select_icon = 'FILEBROWSER'
        row.operator(MOCAPTK_OT_Open_File_Browser.bl_idname, icon=select_icon, text="")
        col.operator(MotionCaptureToolkit.bl_idname, text="Import")

class MOCAPTK_PT_MotionCaptureLoopPanel(MotionCaptureToolkitPanel, bpy.types.Panel):
    bl_parent_id = "MOCAPTK_PT_input_panel"
    bl_idname = "MOCAPTK_PT_loop_panel"
    bl_label = "Loop Controls"
    def draw(self,context):
        
        if(context.object is not None):
            row = self.layout.row()
            obj_mocap_prop = context.object.motion_capture_properties
            row.alignment = 'RIGHT'
            if(obj_mocap_prop.use_loop):
                col = row.column()
                col.alignment = 'LEFT'
                col.prop(obj_mocap_prop,"use_loop_at_start")
                
                col.enabled = not obj_mocap_prop.is_loc_rot_backed_up
            row.prop(obj_mocap_prop,"use_loop")
            
            if(obj_mocap_prop.use_loop):
                col = self.layout.column(align=True)
                row = col.split(factor=0.5,align=True)
                row.alignment = 'RIGHT'
                row.label(text="Loop Start")
                row.alignment = 'LEFT'
                row.prop(obj_mocap_prop,"start_loop")
                row = col.split(factor=0.5,align=True)
                row.alignment = 'RIGHT'
                row.label(text="End")
                row.prop(obj_mocap_prop,"end_loop")

                col.enabled = not obj_mocap_prop.is_loc_rot_backed_up

                row = self.layout.row()

                row.enabled = not obj_mocap_prop.is_loc_rot_backed_up

                col = self.layout.column(align=True)
                row = col.split(factor=0.25,align=True)
                row.separator()

                box = row.box()
                # Currently just works with hip
                box.prop(obj_mocap_prop,"bone_names_enum",text="Lock: ")
                row_inner = box.row()
                col = row_inner.column(align=True)
                col.label(text="Position")
                #Todo: Root only
                pos_axis = ["X","Y","Z"]
                for index, axis in enumerate(pos_axis):
                    if obj_mocap_prop.lock_pos[index] is True:
                        lock_icon = "LOCKED"
                    else:
                        lock_icon = "UNLOCKED"
                    row2 = col.row(align=True)
                    row2.prop(obj_mocap_prop, "lock_pos", index = index, toggle = True, text = axis)
                    row2.label(icon = lock_icon)
                quat_axis = ["W","X","Y","Z"]
                col = row_inner.column(align=True)
                col.label(text="Rotation")
                for index, axis in enumerate(quat_axis):
                    if obj_mocap_prop.lock_rot[index] is True:
                        lock_icon = "LOCKED"
                    else:
                        lock_icon = "UNLOCKED"
                    row2 = col.row(align=True)
                    row2.prop(obj_mocap_prop, "lock_rot", index = index, toggle = True, text = axis)
                    row2.label(icon = lock_icon)
                
                
                row = box.row()
                row.operator(MOCAPTK_OT_Free_Backup_Cache.bl_idname, text="Free Cache")
                row.enabled = obj_mocap_prop.is_loc_rot_backed_up

                col = self.layout.column()
                row = col.split(factor=0.25)

                row.separator()
                row.alignment = 'RIGHT'
                row.operator(MOCAPTK_OT_Append_First_Keyframe.bl_idname, text="Force Loop")

                col = self.layout.column()
                row = col.split(factor=0.25)
                row.separator()
                row.prop(obj_mocap_prop,"force_loop_offset",text="Offset")
                
                #Todo: Offset


        else:
            self.layout.label(text="No armature selected")

    def get_default_scene_start(self,context):
        return context.scene.frame_start
    def get_default_scene_end(self,context):
        return context.scene.frame_end

class MOCAPTK_PT_MotionCaptureCleanKeyPanel(MotionCaptureToolkitPanel, bpy.types.Panel):
    bl_parent_id = "MOCAPTK_PT_input_panel"
    bl_idname = "MOCAPTK_PT_clean_key"
    bl_label = "Keyframe Cleanup"
    def draw(self,context):
        if(context.object is not None):
            row = self.layout.row()
            obj_mocap_prop = context.object.motion_capture_properties
            row.label(text="Simplify Keyframes")
            row = self.layout.row()
            # button for blender built-in, and for fcurves approach
            row.prop(obj_mocap_prop, "clean_threshold", text="Threshold", slider=True)

            row = self.layout.row()
            row.operator(MOCAPTK_OT_Cleanup_Keyframes.bl_idname, text="Simplify Keyframes (Blender)")
            row = self.layout.row()
            row.operator(GRAPH_OT_simplify.bl_idname, text="Simplify Keyframes (SimpliPoly)")

class MOCAPTK_PT_MotionCaptureAdjustKeyPanel(MotionCaptureToolkitPanel, bpy.types.Panel):
    bl_parent_id = "MOCAPTK_PT_input_panel"
    bl_idname = "MOCAPTK_PT_adjust_key"
    bl_label = "Modify Motions"
    def draw(self,context):
        if(context.object is not None):
            row = self.layout.row()
            col = self.layout.column()
            obj_mocap_prop = context.object.motion_capture_properties
            col.label(text="Modify Keyframe Motion")
            #Todo: figure out this UI
            # col.operator(OT_Cleanup_Keyframes.bl_idname, text="Simplify Keyframes (Blender)")
            # col.operator(OT_Cleanup_Keyframes.bl_idname, text="Simplify Keyframes (SimpliPoly)")

def menu_func(self, context):
    self.layout.operator(MotionCaptureToolkit.bl_idname)


class MOCAPTK_OT_Open_File_Browser(Operator, ImportHelper):
    "Opens browser for file input."
    bl_idname = "mocaptk.open_filebrowser"
    bl_label = "Accept"
    
    
    filter_glob: StringProperty(
        default='*.bvh;',
        options={'HIDDEN'}
    )
    
    use_quaternion: BoolProperty(
        name='Use Quaternion',
        description='Must import as quaternion for rotation tools to work correctly (currently)',
        default=True,
    )
    import_scale: FloatProperty(
        name='Scale',
        description='Scale of imported armature.',
        default=1.0,
    )

    def execute(self, context):
        """Do something with the selected file(s)."""

        filename, extension = os.path.splitext(self.filepath)
        
        print('Selected file:', self.filepath)
        print('File name:', filename)
        print('File extension:', extension)
        context.scene.motion_capture_properties.file_path = self.filepath
        context.scene.motion_capture_properties.use_quaternion = self.use_quaternion
        context.scene.motion_capture_properties.import_scale = self.import_scale
        return {'FINISHED'}


class MOCAPTK_OT_Update_Lock(Operator):
    # Currently not needed!
    "Update which axis the loop should be on"
    bl_idname = "mocaptk.update_lock"
    bl_label = "Update Lock"
    bl_options = {'UNDO'}
	
    def execute(self, context):
        pass
        return {'FINISHED'}
class MOCAPTK_OT_Free_Backup_Cache(Operator):
    "Frees backup cache so it can be reallocated"
    bl_idname = "mocaptk.free_backup_cache"
    bl_label = "Free Backup Cache"
    bl_options = {'UNDO'}

    def execute(self, context):
        obj_mocap_prop = context.object.motion_capture_properties
        obj_mocap_prop.lock_pos = [False,False,False]
        obj_mocap_prop.lock_rot = [False,False,False,False]
        obj_mocap_prop.is_loc_rot_backed_up = False
        return {'FINISHED'}
class MOCAPTK_OT_Set_Loop_Start_End_From_Action(Operator):
    "Sets first and last keyframe to the action's first and last keyframe"
    bl_idname = "mocaptk.set_loop_start_end_from_action"
    bl_label = "Set Loop Start/End From Action"
    
    def execute(self, context):
        obj_mocap_prop = context.object.motion_capture_properties
        obj_mocap_prop.start_loop = int(context.object.animation_data.action.frame_range[0])
        obj_mocap_prop.end_loop = int(context.object.animation_data.action.frame_range[1])
        return {'FINISHED'}

class MOCAPTK_OT_Append_First_Keyframe(Operator):
    "Adds the first keyframe to after the last keyframe (Warning! Currently destructive)"
    bl_idname = "mocaptk.append_first_keyframe"
    bl_label = "Append First Keyframe"

    def execute(self, context):
        obj_mocap_prop = context.object.motion_capture_properties
        anim_fcurves = context.object.animation_data.action.fcurves
        loop_start = obj_mocap_prop.start_loop
        loop_end = obj_mocap_prop.end_loop
        offset = obj_mocap_prop.force_loop_offset
        # Copy all keyframes in fcurve at loop_start and insert at loop_end
        # Remove all keyframes between loop_end and loop_end + offset

        for fcurve in anim_fcurves:
            for keyframe in fcurve.keyframe_points:
                # Warning: frame time is stored as float, so we need to cast to int

                if int(keyframe.co[0]) > loop_end and int(keyframe.co[0]) < loop_end + offset:
                    fcurve.keyframe_points.remove(keyframe)
            fcurve.keyframe_points.insert(frame=loop_end + offset, value=fcurve.evaluate(loop_start))
            # Update scene
            context.view_layer.update()
            # Remove keyframes after loop_end + offset
            # THIS ISN"T WORKING SMH
            # for keyframe in fcurve.keyframe_points:
            #     if keyframe.co[0] > loop_end + offset:
            #         fcurve.keyframe_points.remove(keyframe)
            
        
            
            
                
        
        return {'FINISHED'}

# Ref for exaggerating motion:

# def move_kp(obj, dx):
#     anim_data           = obj.animation_data
#     if not anim_data:   return

#     action              = anim_data.action
#     if not action:      return

#     for fc in action.fcurves:
#         for kp in fc.keyframe_points:
#             kp.co[0]            += dx
#             kp.handle_left[0]   += dx
#             kp.handle_right[0]  += dx

class MOCAPTK_OT_Toggle_Root_Lock(Operator):
    "Locks in place (current behaviour" \
    "is to always lock relative to the start time," \
    "could be relative to current frame later)"
    bl_idname = "mocaptk.toggle_root_lock"
    bl_label = "Toggle Root Lock"
    bl_options = {'UNDO'}

    def allocate_backup(self,col,start,finish):
        col.clear()
        for i in range(finish-start + 2):
            col.add()
        print("Allocated backup: " + str(len(col)) + " frames")
    # todo: need to revisit start frame and end frame
    def backup_keyframe(self,context,target,isRotation = False):
        currentFrame = context.scene.frame_current
        start_frame = context.object.motion_capture_properties.start_loop
        end_frame = context.object.motion_capture_properties.end_loop
        backup_col = context.object.motion_capture_collections

        context.scene.frame_set(start_frame)
        # todo: partially implemented for dynamically backing up only what needs to be
        # (tricky bc cannot copy.copy the data)
        # if isRotation is False:
        #     backup_lock = context.object.motion_capture_properties.lock_pos
        #     tmpLock = [False,False,False]
        # else:
        #     backup_lock = context.object.motion_capture_properties.lock_rot
        #     tmpLock = [False,False,False,False]
        
        #unlock so keyframes can be backed up correctly
        # self.lock_keyframe(context,target,isRotation)
        # lock is dependent on backup so this needs to be restructed, otherwise gets into loop (also whoops)
        for i in range(end_frame-start_frame + 2):
            context.scene.frame_set(i)
            if isRotation is False:
                backup_col[i].root_loc_backup = target.location
            else:
                backup_col[i].root_rot_backup = target.rotation_quaternion
        context.scene.frame_set(currentFrame)
        context.view_layer.update()

    def lock_keyframe(self,context,target,isRotation = False):
        currentFrame = context.scene.frame_current
        start_frame = context.object.motion_capture_properties.start_loop
        end_frame = context.object.motion_capture_properties.end_loop
        backup_col = context.object.motion_capture_collections

        if context.object.motion_capture_properties.use_loop_at_start is True:
            context.scene.frame_set(start_frame)
        
        
        if not isRotation:
            keyframe_type = "location"
            keyframe_length = 3
            initialTargetValue = copy.copy(target.location)
            keyframe_target = target.location
            keyframe_lock = context.object.motion_capture_properties.lock_pos
        else:
            keyframe_type = "rotation_quaternion"
            keyframe_length = 4
            initialTargetValue = copy.copy(target.rotation_quaternion)
            keyframe_target = target.rotation_quaternion
            keyframe_lock = context.object.motion_capture_properties.lock_rot
        
        for i in range(end_frame-start_frame + 2):
            context.scene.frame_set(i)
            if isRotation is False:
                target_col = backup_col[i].root_loc_backup
            else:
                target_col = backup_col[i].root_rot_backup
            for j in range(0,keyframe_length):
                if keyframe_lock[j] is True:
                    keyframe_target[j] = initialTargetValue[j]
                else:
                    keyframe_target[j] = target_col[j]
                    #print("Setting keyframe at frame " + str(i) + " for " + keyframe_type + " to " + str(keyframe_target[j]))
            target.keyframe_insert(keyframe_type,frame=i)
        context.scene.frame_set(currentFrame)
        context.view_layer.update()

    def get_root_pose_bone(self, context):
        # Assumes the bone in position 0 is the root (this is always true unless multiple roots?)
        return context.object.pose.bones[0]
    
    def execute(self, context):
        obj_mocap_prop = context.object.motion_capture_properties
        col_mocap_prop = context.object.motion_capture_collections
        start_loop = obj_mocap_prop.start_loop
        end_loop = obj_mocap_prop.end_loop
        is_backed_up = obj_mocap_prop.is_loc_rot_backed_up
        rootpb = self.get_root_pose_bone(context)
        


        #todo: reset back up if start/end loop changes
        #todo: better integrate if undoing parts of what is locked
        if(is_backed_up is False):
            self.allocate_backup(col_mocap_prop,start_loop,end_loop)
            self.backup_keyframe(context,rootpb)
            self.backup_keyframe(context,rootpb,True)
            obj_mocap_prop.is_loc_rot_backed_up = True
        
        self.lock_keyframe(context,rootpb)
        self.lock_keyframe(context,rootpb,True)
        return {'FINISHED'}
    
class MOCAPTK_OT_Cleanup_Keyframes(Operator):
    # Currently not needed!
    "Clean up duplicate keyframes (Blender's default clean)"
    bl_idname = "mocaptk.clean_keys"
    bl_label = "Clean Keys"
    bl_options = {'UNDO'}

    def select_all_bones(self, context):
        for bone in context.object.pose.bones:
            bone.bone.select = True
	
    def execute(self, context):
        clean_threshold = context.object.motion_capture_properties.clean_threshold
        if(context.object.animation_data.action is None):
            print("No selected action found!")
            return {'FINISHED'}
        else:
            # select action
            self.select_all_bones(context)
        area = [area for area in bpy.context.screen.areas if area.type == "DOPESHEET_EDITOR"][0]
        with bpy.context.temp_override(area=area):
            bpy.ops.action.clean(threshold = clean_threshold, channels = False)
        return {'FINISHED'}
    

def simplypoly(splineVerts, options):
    # main vars
    newVerts = []           # list of vertindices to keep
    points = splineVerts    # list of 3dVectors
    pointCurva = []         # table with curvatures
    curvatures = []         # averaged curvatures per vert
    for p in points:
        pointCurva.append([])
    order = options[3]      # order of sliding beziercurves
    k_thresh = options[2]   # curvature threshold
    dis_error = options[6]  # additional distance error

    # get curvatures per vert
    for i, point in enumerate(points[: -(order - 1)]):
        BVerts = points[i: i + order]
        for b, BVert in enumerate(BVerts[1: -1]):
            deriv1 = getDerivative(BVerts, 1 / (order - 1), order - 1)
            deriv2 = getDerivative(BVerts, 1 / (order - 1), order - 2)
            curva = getCurvature(deriv1, deriv2)
            pointCurva[i + b + 1].append(curva)

    # average the curvatures
    for i in range(len(points)):
        avgCurva = sum(pointCurva[i]) / (order - 1)
        curvatures.append(avgCurva)

    # get distancevalues per vert - same as Ramer-Douglas-Peucker
    # but for every vert
    distances = [0.0]  # first vert is always kept
    for i, point in enumerate(points[1: -1]):
        dist = altitude(points[i], points[i + 2], points[i + 1])
        distances.append(dist)
    distances.append(0.0)  # last vert is always kept

    # generate list of vert indices to keep
    # tested against averaged curvatures and distances of neighbour verts
    newVerts.append(0)  # first vert is always kept
    for i, curv in enumerate(curvatures):
        if (curv >= k_thresh * 0.01 or distances[i] >= dis_error * 0.1):
            newVerts.append(i)
    newVerts.append(len(curvatures) - 1)  # last vert is always kept

    return newVerts


# get binomial coefficient
def binom(n, m):
    b = [0] * (n + 1)
    b[0] = 1
    for i in range(1, n + 1):
        b[i] = 1
        j = i - 1
        while j > 0:
            b[j] += b[j - 1]
            j -= 1
    return b[m]


# get nth derivative of order(len(verts)) bezier curve
def getDerivative(verts, t, nth):
    order = len(verts) - 1 - nth
    QVerts = []

    if nth:
        for i in range(nth):
            if QVerts:
                verts = QVerts
            derivVerts = []
            for i in range(len(verts) - 1):
                derivVerts.append(verts[i + 1] - verts[i])
            QVerts = derivVerts
    else:
        QVerts = verts

    if len(verts[0]) == 3:
        point = Vector((0, 0, 0))
    if len(verts[0]) == 2:
        point = Vector((0, 0))

    for i, vert in enumerate(QVerts):
        point += binom(order, i) * pow(t, i) * pow(1 - t, order - i) * vert
    deriv = point

    return deriv


# get curvature from first, second derivative
def getCurvature(deriv1, deriv2):
    if deriv1.length == 0:  # in case of points in straight line
        curvature = 0
        return curvature
    curvature = (deriv1.cross(deriv2)).length / pow(deriv1.length, 3)
    return curvature


# ### Ramer-Douglas-Peucker algorithm ###
# Implementation adapted from similar approach by Michael Soluyanov
def altitude(point1, point2, pointn):
    edge1 = point2 - point1
    edge2 = pointn - point1
    if edge2.length == 0:
        altitude = 0
        return altitude
    if edge1.length == 0:
        altitude = edge2.length
        return altitude
    alpha = edge1.angle(edge2)
    altitude = sin(alpha) * edge2.length
    return altitude


def iterate(points, newVerts, error):
    new = []
    for newIndex in range(len(newVerts) - 1):
        bigVert = 0
        alti_store = 0
        for i, point in enumerate(points[newVerts[newIndex] + 1: newVerts[newIndex + 1]]):
            alti = altitude(points[newVerts[newIndex]], points[newVerts[newIndex + 1]], point)
            if alti > alti_store:
                alti_store = alti
                if alti_store >= error:
                    bigVert = i + 1 + newVerts[newIndex]
        if bigVert:
            new.append(bigVert)
    if new == []:
        return False
    return new


def simplify_RDP(splineVerts, options):
    # main vars
    error = options[4]

    # set first and last vert
    newVerts = [0, len(splineVerts) - 1]

    # iterate through the points
    new = 1
    while new is not False:
        new = iterate(splineVerts, newVerts, error)
        if new:
            newVerts += new
            newVerts.sort()
    return newVerts
    
# get array of new coords for new spline from vertindices
def vertsToPoints(newVerts, splineVerts, splineType):
    newPoints = []
    if splineType == 'BEZIER':
        for v in newVerts:
            newPoints += splineVerts[v].to_tuple()

    # array for nonBEZIER output
    else:
        for v in newVerts:
            newPoints += (splineVerts[v].to_tuple())
            if splineType == 'NURBS':
                newPoints.append(1)  # for nurbs w = 1
            else:                    # for poly w = 0
                newPoints.append(0)
    return newPoints

def main(context, obj, options, curve_dimension):
    output = options[1]
    degreeOut = options[5]
    keepShort = options[7]
    bpy.ops.object.select_all(action='DESELECT')
    scene = context.scene
    splines = obj.data.splines.values()

    # create curvedatablock
    curve = bpy.data.curves.new("Simple_" + obj.name, type='CURVE')
    curve.dimensions = curve_dimension

    # go through splines
    for spline_i, spline in enumerate(splines):
        # test if spline is a long enough
        if len(spline.points) >= 3 or keepShort:
            # check what type of spline to create
            if output == 'INPUT':
                splineType = spline.type
            else:
                splineType = output

            # get vec3 list to simplify
            if spline.type == 'BEZIER':  # get bezierverts
                splineVerts = [splineVert.co.copy()
                               for splineVert in spline.bezier_points.values()]

            else:  # verts from all other types of curves
                splineVerts = [splineVert.co.to_3d()
                               for splineVert in spline.points.values()]

            newVerts = simplify_RDP(splineVerts, options)
            newPoints = vertsToPoints(newVerts, splineVerts, splineType)
            newSpline = curve.splines.new(type=splineType)

            # put newPoints into spline according to type
            if splineType == 'BEZIER':
                newSpline.bezier_points.add(int(len(newPoints) * 0.33))
                newSpline.bezier_points.foreach_set('co', newPoints)
            else:
                newSpline.points.add(int(len(newPoints) * 0.25 - 1))
                newSpline.points.foreach_set('co', newPoints)

            # set degree of outputNurbsCurve
            if output == 'NURBS':
                newSpline.order_u = degreeOut

            # splineoptions
            newSpline.use_endpoint_u = spline.use_endpoint_u

    # create new object and put into scene
    newCurve = bpy.data.objects.new("Simple_" + obj.name, curve)
    coll = context.view_layer.active_layer_collection.collection
    coll.objects.link(newCurve)
    newCurve.select_set(True)

    context.view_layer.objects.active = newCurve
    newCurve.matrix_world = obj.matrix_world

    # set bezierhandles to auto
    for spline in newCurve.data.splines:
        for p in spline.bezier_points:
            p.handle_left_type = 'AUTO'
            p.handle_right_type = 'AUTO'

    return


def getFcurveData(obj):
    fcurves = []
    for fc in obj.animation_data.action.fcurves:
        if fc.select:
            fcVerts = [vcVert.co.to_3d()
                       for vcVert in fc.keyframe_points.values()]
            fcurves.append(fcVerts)
    return fcurves


def selectedfcurves(obj):
    fcurves_sel = []
    for i, fc in enumerate(obj.animation_data.action.fcurves):
        if fc.select:
            fcurves_sel.append(fc)
    return fcurves_sel


def fcurves_simplify(context, obj, options, fcurves):
    fcurve_sel = selectedfcurves(obj)

    for fcurve_i, fcurve in enumerate(fcurves):
        # 3 for sliding window bezier fit
        if len(fcurve) >= 3:
            newVerts = simplify_RDP(fcurve, options)
            newPoints = []
            for v in newVerts:
                newPoints.append(fcurve[v])
            for i in range(len(fcurve) - 1, 0, -1):
                fcurve_sel[fcurve_i].keyframe_points.remove(fcurve_sel[fcurve_i].keyframe_points[i])
            for v in newPoints:
                fcurve_sel[fcurve_i].keyframe_points.insert(frame=v[0], value=v[1])
    return

# Adapted from similar approach by Michael Soluyanov, can probably be trimmed down more
class GRAPH_OT_simplify(Operator):
    bl_idname = "graph.simplify"
    bl_label = "Simplify F-Curves"
    bl_description = ("Simplify selected Curves\n")
    bl_options = {'REGISTER', 'UNDO'}

    opModes = [
            ('DISTANCE', 'Distance', 'Distance-based simplification (Poly)'),
            ('CURVATURE', 'Curvature', 'Curvature-based simplification (RDP)')]
    mode: EnumProperty(
            name="Mode",
            description="Choose algorithm to use",
            items=opModes
            )
    k_thresh: FloatProperty(
            name="k",
            min=0, soft_min=0,
            default=0, precision=5,
            description="Threshold"
            )
    pointsNr: IntProperty(
            name="n",
            min=5, soft_min=5,
            max=16, soft_max=9,
            default=5,
            description="Degree of curve to get averaged curvatures"
            )
    error: FloatProperty(
            name="Error",
            description="Maximum allowed distance error",
            min=0.0, soft_min=0.0,
            default=0, precision=5,
            step = 0.1
            )
    degreeOut: IntProperty(
            name="Degree",
            min=3, soft_min=3,
            max=7, soft_max=7,
            default=5,
            description="Degree of new curve"
            )
    dis_error: FloatProperty(
            name="Distance error",
            description="Maximum allowed distance error in Blender Units",
            min=0, soft_min=0,
            default=0.0, precision=5
            )
    fcurves = []

    def draw(self, context):
        layout = self.layout
        col = layout.column()

        col.label(text="Distance Error:")
        col.prop(self, "error", expand=True)

    @classmethod
    def poll(cls, context):
        # Check for animdata
        obj = context.active_object
        fcurves = False
        if obj:
            animdata = obj.animation_data
            if animdata:
                act = animdata.action
                if act:
                    fcurves = act.fcurves
        return (obj and fcurves)

    def execute(self, context):
        options = [
                self.mode,       # 0
                self.mode,       # 1
                self.k_thresh,   # 2
                self.pointsNr,   # 3
                self.error,      # 4
                self.degreeOut,  # 6
                self.dis_error   # 7
                ]

        obj = context.active_object

        if not self.fcurves:
            self.fcurves = getFcurveData(obj)

        fcurves_simplify(context, obj, options, self.fcurves)

        return {'FINISHED'}
classes = (
    MotionCaptureToolkit,
    MotionCaptureSceneProperties,
    MotionCaptureObjectProperties,
    MotionCaptureObjectCollections,
    MOCAPTK_PT_MotionCaptureInputFilePanel,
    MOCAPTK_PT_MotionCaptureLoopPanel,
    MOCAPTK_PT_MotionCaptureCleanKeyPanel,
    MOCAPTK_PT_MotionCaptureAdjustKeyPanel,
    MOCAPTK_OT_Open_File_Browser,
    MOCAPTK_OT_Update_Lock,
    MOCAPTK_OT_Toggle_Root_Lock,
    MOCAPTK_OT_Free_Backup_Cache,
    MOCAPTK_OT_Cleanup_Keyframes,
    MOCAPTK_OT_Set_Loop_Start_End_From_Action,
    MOCAPTK_OT_Append_First_Keyframe,
    GRAPH_OT_simplify,
)
register_classes, unregister_classes = bpy.utils.register_classes_factory(classes)

def register():
    register_classes()
    
    bpy.types.Scene.motion_capture_properties = bpy.props.PointerProperty(type=MotionCaptureSceneProperties)
    bpy.types.Object.motion_capture_properties = bpy.props.PointerProperty(type=MotionCaptureObjectProperties)
    bpy.types.Object.motion_capture_collections = bpy.props.CollectionProperty(type=MotionCaptureObjectCollections)
                                                        
  # Adds the new operator to an existing menu.

def unregister():
    unregister_classes()
    del bpy.types.Scene.motion_capture_properties
    del bpy.types.Object.motion_capture_properties
    del bpy.types.Object.motion_capture_collections

if __name__ == "__main__":
    register()
