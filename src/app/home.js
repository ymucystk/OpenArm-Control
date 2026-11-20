"use client";
import 'aframe'
import * as React from 'react'
const THREE = window.AFRAME.THREE; // これで　AFRAME と　THREEを同時に使える
import Controller from './controller.js'

const joint_pos = {
  body_link0:{x:0,y:0,z:0},
  base_link_left:{x:0.03,y:0.697,z:0},
  base_link_right:{x:-0.03,y:0.697,z:0},
  link1_left:{x:0.062,y:0.0,z:0},
  link1_right:{x:-0.062,y:0.0,z:0},
  link2_left:{x:0.06,y:0.0,z:0},
  link2_right:{x:-0.06,y:0.0,z:0},
  link3_left:{x:0,y:-0.065,z:0},
  link3_right:{x:0,y:-0.065,z:0},
  link4_left:{x:0,y:-0.15375,z:0},
  link4_right:{x:0,y:-0.15375,z:0},
  link5_left:{x:0,y:-0.095,z:0},
  link5_right:{x:0,y:-0.095,z:0},
  link6_left:{x:0,y:-0.1205,z:0},
  link6_right:{x:0,y:-0.1205,z:0},
  gripper_base_left:{x:0,y:-0,z:0.0205},
  gripper_base_right:{x:0,y:-0,z:0.0205},
  finger_inner_left:{x:0.018,y:-0.068,z:-0.022},
  finger_inner_right:{x:0.018,y:-0.068,z:-0.022},
  finger_outer_left:{x:-0.018,y:-0.068,z:-0.022},
  finger_outer_right:{x:-0.018,y:-0.068,z:-0.022},
}

let registered = false
const x_vec_base = new THREE.Vector3(1,0,0).normalize()
const y_vec_base = new THREE.Vector3(0,1,0).normalize()
const z_vec_base = new THREE.Vector3(0,0,1).normalize()

const object3D_map = {}
const rotate_table = {}
const rightArmJoints = [
  { name: 'link1_right', axis: x_vec_base },
  { name: 'link2_right', axis: z_vec_base },
  { name: 'link3_right', axis: y_vec_base },
  { name: 'link4_right', axis: x_vec_base },
  { name: 'link5_right', axis: y_vec_base },
  { name: 'link6_right', axis: x_vec_base },
  { name: 'gripper_base_right', axis: z_vec_base },
]
const rightArmActuatedJoints = rightArmJoints.slice(0, -1)
const rightArmJointNames = rightArmActuatedJoints.map(({ name }) => name)
const rightArmPositionJoints = rightArmActuatedJoints.slice(0, -1)
const rightArmVr = {
  inVr: false,
  active: false,
  angles: Object.fromEntries(rightArmJointNames.map((name) => [name, 0])),
  positionOffset: new THREE.Vector3(),
  rotationOffset: new THREE.Quaternion(),
  gripLevel: 0,
  lastSyncTime: 0,
}
let syncRightArmControls = () => {}
let target_move_distance = 0
const target_move_speed = (1000 / 0.5)
const max_move_unit = {
  body_link0:(1/500),
  base_link_left:(1/500),
  base_link_right:(1/500),
  link1_left:(1/500),
  link1_right:(1/500),
  link2_left:(1/500),
  link2_right:(1/500),
  link3_left:(1/500),
  link3_right:(1/500),
  link4_left:(1/500),
  link4_right:(1/500),
  link5_left:(1/500),
  link5_right:(1/500),
  link6_left:(1/500),
  link6_right:(1/500),
  gripper_base_left:(1/500),
  gripper_base_right:(1/500),
  finger_inner_left:(1/500),
  finger_inner_right:(1/500),
  finger_outer_left:(1/500),
  finger_outer_right:(1/500),
}

function useRefState(initialValue=undefined) {
  const ref0 = React.useRef(initialValue);
  const ref1 = React.useRef(initialValue);
  const reftbl = [ref0,ref1]
  const idx = React.useRef(0);
  const rtnval = React.useRef(reftbl[idx.current].current);
  const prevrtnval = React.useRef(reftbl[idx.current].current);
  function setValue(arg){
    const newidx = idx.current===0?1:0
    if (typeof arg === 'function') {
      reftbl[newidx].current = arg(reftbl[idx.current].current)
    }else{
      reftbl[newidx].current = arg
    }
    rtnval.current = reftbl[newidx].current
    prevrtnval.current = reftbl[idx.current].current
    idx.current = newidx
  }
  return [rtnval.current, setValue, rtnval, prevrtnval.current];
}

function solveLinearSystem(matrix, vector) {
  const size = vector.length
  const augmented = matrix.map((row, index) => [...row, vector[index]])

  for (let column = 0; column < size; column += 1) {
    let pivot = column
    for (let row = column + 1; row < size; row += 1) {
      if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column])) {
        pivot = row
      }
    }
    if (Math.abs(augmented[pivot][column]) < 1e-10) {
      return Array(size).fill(0)
    }
    const pivotRow = augmented[column]
    augmented[column] = augmented[pivot]
    augmented[pivot] = pivotRow

    const divisor = augmented[column][column]
    for (let entry = column; entry <= size; entry += 1) {
      augmented[column][entry] /= divisor
    }
    for (let row = 0; row < size; row += 1) {
      if (row === column) continue
      const factor = augmented[row][column]
      for (let entry = column; entry <= size; entry += 1) {
        augmented[row][entry] -= factor * augmented[column][entry]
      }
    }
  }
  return augmented.map((row) => row[size])
}

function updateRightArmIk(targetPosition, targetRotation, deltaSeconds) {
  const endEffector = object3D_map.link6_right
  if (!endEffector || rightArmActuatedJoints.some(({ name }) => !object3D_map[name])) {
    return false
  }

  const damping = 0.1
  const iterations = 5
  const maxJointSpeed = Math.PI
  const frameTime = Number.isFinite(deltaSeconds) && deltaSeconds > 0
    ? Math.max(1 / 120, Math.min(deltaSeconds, 1 / 20))
    : 1 / 60
  const stepLimit = maxJointSpeed * frameTime / iterations
  const allJointIndices = rightArmPositionJoints.map((_, index) => index)
  const link4Index = rightArmPositionJoints.findIndex(({ name }) => name === 'link4_right')
  const solveJointSteps = (jacobian, error, jointIndices) => {
    const normalMatrix = Array.from({ length: 3 }, (_, row) =>
      Array.from({ length: 3 }, (_, column) => {
        let value = row === column ? damping * damping : 0
        for (const jointIndex of jointIndices) {
          value += jacobian[row][jointIndex] * jacobian[column][jointIndex]
        }
        return value
      }),
    )
    const correction = solveLinearSystem(normalMatrix, error)
    const steps = Array(rightArmPositionJoints.length).fill(0)
    for (const jointIndex of jointIndices) {
      steps[jointIndex] = Math.max(-stepLimit, Math.min(stepLimit,
        jacobian.reduce((sum, row, rowIndex) => sum + row[jointIndex] * correction[rowIndex], 0)))
    }
    return steps
  }

  for (let iteration = 0; iteration < iterations; iteration += 1) {
    object3D_map.body_link0.updateWorldMatrix(true, true)
    const endPosition = endEffector.getWorldPosition(new THREE.Vector3())
    const positionError = targetPosition.clone().sub(endPosition)
    const error = [positionError.x, positionError.y, positionError.z]
    const jacobian = Array.from({ length: 3 }, () => Array(rightArmPositionJoints.length).fill(0))
    for (let jointIndex = 0; jointIndex < rightArmPositionJoints.length; jointIndex += 1) {
      const { name, axis } = rightArmPositionJoints[jointIndex]
      const joint = object3D_map[name]
      const jointPosition = joint.getWorldPosition(new THREE.Vector3())
      const worldAxis = axis.clone().applyQuaternion(joint.getWorldQuaternion(new THREE.Quaternion())).normalize()
      const linear = worldAxis.clone().cross(endPosition.clone().sub(jointPosition))
      jacobian[0][jointIndex] = linear.x
      jacobian[1][jointIndex] = linear.y
      jacobian[2][jointIndex] = linear.z
    }

    const steps = solveJointSteps(jacobian, error, allJointIndices)
    const link4Angle = rightArmVr.angles.link4_right
    const nextLink4Angle = link4Angle + toAngle(steps[link4Index])
    if (nextLink4Angle < -180 || nextLink4Angle > 0) {
      const boundedAngle = Math.max(-180, Math.min(0, nextLink4Angle))
      const link4Step = toRadian(boundedAngle - link4Angle)
      const remainingError = error.map((value, row) => value - jacobian[row][link4Index] * link4Step)
      const freeJointIndices = allJointIndices.filter((index) => index !== link4Index)
      const adjustedSteps = solveJointSteps(jacobian, remainingError, freeJointIndices)
      adjustedSteps[link4Index] = link4Step
      for (let jointIndex = 0; jointIndex < steps.length; jointIndex += 1) {
        steps[jointIndex] = adjustedSteps[jointIndex]
      }
    }

    for (let jointIndex = 0; jointIndex < rightArmPositionJoints.length; jointIndex += 1) {
      const { name, axis } = rightArmPositionJoints[jointIndex]
      const nextAngle = rightArmVr.angles[name] + toAngle(steps[jointIndex])
      rightArmVr.angles[name] = name === 'link4_right'
        ? Math.max(-180, Math.min(0, nextAngle))
        : normalize180(nextAngle)
      object3D_map[name].quaternion.setFromAxisAngle(axis, toRadian(rightArmVr.angles[name]))
    }
  }
  object3D_map.body_link0.updateWorldMatrix(true, true)

  const link6Joint = object3D_map.link6_right
  const parentRotation = link6Joint.parent.getWorldQuaternion(new THREE.Quaternion())
  const localTargetRotation = parentRotation.invert().multiply(targetRotation)
  const twistMagnitude = Math.hypot(localTargetRotation.x, localTargetRotation.w)
  if (twistMagnitude > 1e-8) {
    const targetAngle = normalize180(
      toAngle(2 * Math.atan2(localTargetRotation.x / twistMagnitude, localTargetRotation.w / twistMagnitude)),
    )
    const maxStepDegrees = toAngle(maxJointSpeed * frameTime)
    const angleDifference = normalize180(targetAngle - rightArmVr.angles.link6_right)
    const step = Math.max(-maxStepDegrees, Math.min(maxStepDegrees, angleDifference))
    rightArmVr.angles.link6_right = normalize180(rightArmVr.angles.link6_right + step)
    link6Joint.quaternion.setFromAxisAngle(x_vec_base, toRadian(rightArmVr.angles.link6_right))
  }
  return true
}

function setRightGripperLevel(level) {
  rightArmVr.gripLevel = Math.max(0, Math.min(1, level))
  if (object3D_map.finger_inner_right) {
    object3D_map.finger_inner_right.quaternion.setFromAxisAngle(
      z_vec_base,
      toRadian(rightArmVr.gripLevel * 30),
    )
  }
  if (object3D_map.finger_outer_right) {
    object3D_map.finger_outer_right.quaternion.setFromAxisAngle(
      z_vec_base,
      toRadian(rightArmVr.gripLevel * -30),
    )
  }
}

function syncRightArmState() {
  syncRightArmControls(rightArmVr.angles, rightArmVr.gripLevel)
  rightArmVr.lastSyncTime = performance.now()
}

export default function Home(props) {
  const [update, set_update] = React.useState(0)
  const [rendered,set_rendered] = useRefState(false)

  const [body_link0_rotate, set_body_link0_rotate ] = useRefState(0)
  const [base_link_left_rotate, set_base_link_left_rotate ] = useRefState(0)
  const [base_link_right_rotate, set_base_link_right_rotate ] = useRefState(0)
  const [link1_left_rotate, set_link1_left_rotate ] = useRefState(0)
  const [link1_right_rotate, set_link1_right_rotate ] = useRefState(0)
  const [link2_left_rotate, set_link2_left_rotate ] = useRefState(0)
  const [link2_right_rotate, set_link2_right_rotate ] = useRefState(0)
  const [link3_left_rotate, set_link3_left_rotate ] = useRefState(0)
  const [link3_right_rotate, set_link3_right_rotate ] = useRefState(0)
  const [link4_left_rotate, set_link4_left_rotate ] = useRefState(0)
  const [link4_right_rotate, set_link4_right_rotate ] = useRefState(0)
  const [link5_left_rotate, set_link5_left_rotate ] = useRefState(0)
  const [link5_right_rotate, set_link5_right_rotate ] = useRefState(0)
  const [link6_left_rotate, set_link6_left_rotate ] = useRefState(0)
  const [link6_right_rotate, set_link6_right_rotate ] = useRefState(0)
  const [gripper_base_left_rotate, set_gripper_base_left_rotate ] = useRefState(0)
  const [gripper_base_right_rotate, set_gripper_base_right_rotate ] = useRefState(0)
  const [finger_inner_left_rotate, set_finger_inner_left_rotate ] = useRefState(0)
  const [finger_inner_right_rotate, set_finger_inner_right_rotate ] = useRefState(0)
  const [finger_outer_left_rotate, set_finger_outer_left_rotate ] = useRefState(0)
  const [finger_outer_right_rotate, set_finger_outer_right_rotate ] = useRefState(0)

  React.useEffect(() => {
    syncRightArmControls = (angles, gripLevel) => {
      set_link1_right_rotate(angles.link1_right)
      set_link2_right_rotate(angles.link2_right)
      set_link3_right_rotate(angles.link3_right)
      set_link4_right_rotate(angles.link4_right)
      set_link5_right_rotate(angles.link5_right)
      set_link6_right_rotate(angles.link6_right)
      set_finger_inner_right_rotate(gripLevel * 30)
      set_finger_outer_right_rotate(gripLevel * -30)
    }
    return () => {
      syncRightArmControls = () => {}
    }
  }, [])

  const [c_pos_x,set_c_pos_x] = useRefState(0)
  const [c_pos_y,set_c_pos_y] = useRefState(0.4)
  const [c_pos_z,set_c_pos_z] = useRefState(-0.6)
  const [c_deg_x,set_c_deg_x] = useRefState(0)
  const [c_deg_y,set_c_deg_y] = useRefState(180)
  const [c_deg_z,set_c_deg_z] = useRefState(0)

  function updateRotation(){
    const joint_keys = Object.keys(rotate_table)
    const key_num = joint_keys.length
    for (let i = 0; i < key_num; i = i + 1) {
      const key = joint_keys[i]
      if (rightArmVr.active && rightArmJointNames.includes(key)) {
        delete rotate_table[key]
        continue
      }
      if (rightArmVr.inVr && (key === 'finger_inner_right' || key === 'finger_outer_right')) {
        delete rotate_table[key]
        continue
      }
      const current_object3D = object3D_map[key]
      if (current_object3D !== undefined) {
        const current_data = rotate_table[key]
        if (current_data.first) {
          current_data.first = false
          current_data.starttime = performance.now()
          current_data.start_quaternion = current_object3D.quaternion.clone()
          const move_time_1 = target_move_distance * target_move_speed
          const wk_euler = new THREE.Quaternion().angleTo(
            current_data.start_quaternion.clone().invert().multiply(current_data.end_quaternion))
          const move_time_2 = (Math.abs(toAngle(wk_euler)) * max_move_unit[key]) * 1000
          current_data.move_time = Math.max(move_time_1, move_time_2)
          current_data.endtime = current_data.starttime + current_data.move_time
        }
        const current_time = performance.now()
        if (current_time < current_data.endtime) {
          const elapsed_time = current_time - current_data.starttime
          current_object3D.quaternion.slerpQuaternions(
            current_data.start_quaternion, current_data.end_quaternion, (elapsed_time / current_data.move_time))
        }else{
          current_object3D.quaternion.copy(current_data.end_quaternion)
          delete rotate_table[key]
        }
      }
    }
  }
 
  const makeQuaternion = (axis,angle)=>new THREE.Quaternion().setFromAxisAngle(axis,toRadian(angle))

  React.useEffect(() => {
    rotate_table.body_link0 = { end_quaternion: makeQuaternion(y_vec_base, body_link0_rotate), first: true }
  },[body_link0_rotate])

  React.useEffect(() => {
    rotate_table.base_link_left = { end_quaternion: makeQuaternion(z_vec_base, base_link_left_rotate), first: true }
  },[base_link_left_rotate])

  React.useEffect(() => {
    rotate_table.base_link_right = { end_quaternion: makeQuaternion(z_vec_base, base_link_right_rotate), first: true }
  },[base_link_right_rotate])

  React.useEffect(() => {
    rotate_table.link1_left = { end_quaternion: makeQuaternion(x_vec_base, link1_left_rotate), first: true }
  },[link1_left_rotate])

  React.useEffect(() => {
    rotate_table.link1_right = { end_quaternion: makeQuaternion(x_vec_base, link1_right_rotate), first: true }
  },[link1_right_rotate])

  React.useEffect(() => {
    rotate_table.link2_left = { end_quaternion: makeQuaternion(z_vec_base, link2_left_rotate), first: true }
  },[link2_left_rotate])

  React.useEffect(() => {
    rotate_table.link2_right = { end_quaternion: makeQuaternion(z_vec_base, link2_right_rotate), first: true }
  },[link2_right_rotate])

  React.useEffect(() => {
    rotate_table.link3_left = { end_quaternion: makeQuaternion(y_vec_base, link3_left_rotate), first: true }
  },[link3_left_rotate])

  React.useEffect(() => {
    rotate_table.link3_right = { end_quaternion: makeQuaternion(y_vec_base, link3_right_rotate), first: true }
  },[link3_right_rotate])

  React.useEffect(() => {
    rotate_table.link4_left = { end_quaternion: makeQuaternion(x_vec_base, link4_left_rotate), first: true }
  },[link4_left_rotate])

  React.useEffect(() => {
    rotate_table.link4_right = { end_quaternion: makeQuaternion(x_vec_base, link4_right_rotate), first: true }
  },[link4_right_rotate])

  React.useEffect(() => {
    rotate_table.link5_left = { end_quaternion: makeQuaternion(y_vec_base, link5_left_rotate), first: true }
  },[link5_left_rotate])

  React.useEffect(() => {
    rotate_table.link5_right = { end_quaternion: makeQuaternion(y_vec_base, link5_right_rotate), first: true }
  },[link5_right_rotate])

  React.useEffect(() => {
    rotate_table.link6_left = { end_quaternion: makeQuaternion(x_vec_base, link6_left_rotate), first: true }
  },[link6_left_rotate])

  React.useEffect(() => {
    rotate_table.link6_right = { end_quaternion: makeQuaternion(x_vec_base, link6_right_rotate), first: true }
  },[link6_right_rotate])

  React.useEffect(() => {
    rotate_table.gripper_base_left = { end_quaternion: makeQuaternion(z_vec_base, gripper_base_left_rotate), first: true }
  },[gripper_base_left_rotate])

  React.useEffect(() => {
    rotate_table.gripper_base_right = { end_quaternion: makeQuaternion(z_vec_base, gripper_base_right_rotate), first: true }
  },[gripper_base_right_rotate])

  React.useEffect(() => {
    rotate_table.finger_inner_left = { end_quaternion: makeQuaternion(z_vec_base, finger_inner_left_rotate), first: true }
  },[finger_inner_left_rotate])

  React.useEffect(() => {
    rotate_table.finger_inner_right = { end_quaternion: makeQuaternion(z_vec_base, finger_inner_right_rotate), first: true }
  },[finger_inner_right_rotate])

  React.useEffect(() => {
    rotate_table.finger_outer_left = { end_quaternion: makeQuaternion(z_vec_base, finger_outer_left_rotate), first: true }
  },[finger_outer_left_rotate])

  React.useEffect(() => {
    rotate_table.finger_outer_right = { end_quaternion: makeQuaternion(z_vec_base, finger_outer_right_rotate), first: true }
  },[finger_outer_right_rotate])

  React.useEffect(() => {
    if(!registered){
      registered = true
      set_rendered(true)
      console.log('set_rendered')
      set_update((v)=>v=v+1)

      AFRAME.registerComponent('obj-id', {
        schema: {type: 'string', default: ''},
        init: function () {
          console.log(this.data)
          if(Object.hasOwn(object3D_map,this.data)){
            object3D_map[this.data] = this.el.object3D
            const wk = joint_pos[this.data]
            this.el.object3D.position.set(wk.x, wk.y, wk.z)
          }else{
            object3D_map[this.data] = this.el.object3D
            const wk = joint_pos[this.data]
            this.el.object3D.position.set(wk.x, wk.y, wk.z)
          }
        },
        remove: function () {
          if(Object.hasOwn(object3D_map,this.data)){
            object3D_map[this.data] = this.el.object3D
            const wk = joint_pos[this.data]
            this.el.object3D.position.set(wk.x, wk.y, wk.z)
          }else{
            object3D_map[this.data] = this.el.object3D
            const wk = joint_pos[this.data]
            this.el.object3D.position.set(wk.x, wk.y, wk.z)
          }
        },
        tick: function (t) {
          updateRotation()
          set_update((v)=>v=v+1)
        }
      });

      AFRAME.registerComponent('scene', {
        schema: { type: 'string', default: '' },
        init: function () {
          console.log('scene init')
        }
      });

      AFRAME.registerComponent('right-arm-vr-control', {
        init: function () {
          this.previousTrigger = false
          this.rig = null
        },
        tick: function (time, timeDelta) {
          const scene = this.el
          const xr = scene.renderer && scene.renderer.xr
          const session = xr && xr.getSession()
          if (!session) {
            if (rightArmVr.active || rightArmVr.inVr) {
              rightArmVr.active = false
              rightArmVr.inVr = false
              syncRightArmState()
            }
            this.previousTrigger = false
            return
          }

          rightArmVr.inVr = true
          const sources = Array.from(session.inputSources)
          const rightIndex = sources.findIndex((source) => source.handedness === 'right')
          if (rightIndex < 0) {
            if (rightArmVr.active) {
              rightArmVr.active = false
              syncRightArmState()
            }
            this.previousTrigger = false
            return
          }

          const inputSource = sources[rightIndex]
          const buttons = inputSource.gamepad && inputSource.gamepad.buttons
          const triggerButton = buttons && buttons[0]
          const gripButton = buttons && buttons[1]
          const triggerPressed = Boolean(triggerButton && (triggerButton.pressed || triggerButton.value > 0.5))
          const gripLevel = gripButton && Number.isFinite(gripButton.value)
            ? Math.max(0, Math.min(1, gripButton.value))
            : Number(Boolean(gripButton && gripButton.pressed))
          const gripChanged = Math.abs(gripLevel - rightArmVr.gripLevel) > 0.001
          setRightGripperLevel(gripLevel)

          if (!triggerPressed) {
            if (rightArmVr.active) {
              rightArmVr.active = false
              syncRightArmState()
            } else if (gripChanged && performance.now() - rightArmVr.lastSyncTime > 100) {
              syncRightArmState()
            }
            this.previousTrigger = false
            return
          }

          const frame = scene.frame || xr.getFrame()
          const referenceSpace = xr.getReferenceSpace()
          const gripPose = inputSource.gripSpace && frame && referenceSpace
            ? frame.getPose(inputSource.gripSpace, referenceSpace)
            : null
          if (!gripPose) return

          if (!this.rig) {
            this.rig = scene.querySelector('#rig')
            if (!this.rig) return
          }
          this.rig.object3D.updateWorldMatrix(true, false)
          const rigRotation = this.rig.object3D.getWorldQuaternion(new THREE.Quaternion())
          const controllerPosition = new THREE.Vector3(
            gripPose.transform.position.x,
            gripPose.transform.position.y,
            gripPose.transform.position.z,
          ).applyMatrix4(this.rig.object3D.matrixWorld)
          const controllerRotation = new THREE.Quaternion(
            gripPose.transform.orientation.x,
            gripPose.transform.orientation.y,
            gripPose.transform.orientation.z,
            gripPose.transform.orientation.w,
          ).premultiply(rigRotation)
          if (!rightArmVr.active || !this.previousTrigger) {
            const endEffector = object3D_map.link6_right
            if (!endEffector) return
            object3D_map.body_link0.updateWorldMatrix(true, true)
            const endPosition = endEffector.getWorldPosition(new THREE.Vector3())
            const endRotation = endEffector.getWorldQuaternion(new THREE.Quaternion())
            rightArmActuatedJoints.forEach(({ name, axis }) => {
              const quaternion = object3D_map[name].quaternion
              const projection = quaternion.x * axis.x + quaternion.y * axis.y + quaternion.z * axis.z
              const angle = normalize180(toAngle(2 * Math.atan2(projection, quaternion.w)))
              rightArmVr.angles[name] = name === 'link4_right'
                ? Math.max(-180, Math.min(0, angle))
                : angle
              object3D_map[name].quaternion.setFromAxisAngle(axis, toRadian(rightArmVr.angles[name]))
              delete rotate_table[name]
            })
            rightArmVr.positionOffset.copy(endPosition.sub(controllerPosition))
            rightArmVr.rotationOffset.copy(controllerRotation.clone().invert().multiply(endRotation))
            rightArmVr.active = true
          }

          const targetPosition = controllerPosition.clone().add(rightArmVr.positionOffset)
          const targetRotation = controllerRotation.multiply(rightArmVr.rotationOffset)
          const ikUpdated = updateRightArmIk(targetPosition, targetRotation, timeDelta / 1000)
          if ((ikUpdated || gripChanged) && performance.now() - rightArmVr.lastSyncTime > 100) {
            syncRightArmState()
          }
          this.previousTrigger = true
        },
      })
    }
  },[])

  const controllerProps = {
    body_link0_rotate, set_body_link0_rotate,
    base_link_left_rotate, set_base_link_left_rotate,
    base_link_right_rotate, set_base_link_right_rotate,

    link1_left_rotate, set_link1_left_rotate,
    link1_right_rotate, set_link1_right_rotate,
    link2_left_rotate, set_link2_left_rotate,
    link2_right_rotate, set_link2_right_rotate,
    link3_left_rotate, set_link3_left_rotate,
    link3_right_rotate, set_link3_right_rotate,
    link4_left_rotate, set_link4_left_rotate,
    link4_right_rotate, set_link4_right_rotate,
    link5_left_rotate, set_link5_left_rotate,
    link5_right_rotate, set_link5_right_rotate,
    link6_left_rotate, set_link6_left_rotate,
    link6_right_rotate, set_link6_right_rotate,

    gripper_base_left_rotate, set_gripper_base_left_rotate,
    gripper_base_right_rotate, set_gripper_base_right_rotate,
    finger_inner_left_rotate, set_finger_inner_left_rotate,
    finger_inner_right_rotate, set_finger_inner_right_rotate,
    finger_outer_left_rotate, set_finger_outer_left_rotate,
    finger_outer_right_rotate, set_finger_outer_right_rotate,

    c_pos_x,set_c_pos_x,c_pos_y,set_c_pos_y,c_pos_z,set_c_pos_z,
    c_deg_x,set_c_deg_x,c_deg_y,set_c_deg_y,c_deg_z,set_c_deg_z
  }

  const robotProps = { visible:true }

  if(rendered){
    return (
    <>
      <a-scene scene right-arm-vr-control>
        <a-plane position="0 0 0" rotation="-90 0 0" width="10" height="10" color="#7BC8A4" shadow></a-plane>
        <Assets/>
        <Robot_data {...robotProps}/>
        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="1 1 1"></a-entity>
        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="-1 1 1"></a-entity>
        <a-entity light="type: directional; color: #EEE; intensity: 0.25" position="-1 1 -1"></a-entity>
        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="1 1 -1"></a-entity>
        <a-entity light="type: directional; color: #EFE; intensity: 0.05" position="0 -1 0"></a-entity>
        <a-entity id="rig" position={`${c_pos_x} ${c_pos_y} ${c_pos_z}`} rotation={`${c_deg_x} ${c_deg_y} ${c_deg_z}`}>
          <a-camera id="camera" cursor="rayOrigin: mouse;" position="0 0 0" look-controls-enabled="false" wasd-controls-enabled="false"></a-camera>
        </a-entity>
      </a-scene>
      <Controller {...controllerProps}/>
    </>
    );
  }else{
    return(
      <a-scene>
        <Assets/>
      </a-scene>
    )
  }
}

const Assets = ()=>{
  return (
    <a-assets>
      {/*MODEL*/}
      <a-asset-items id="body_link0" src="body_link0.gltf" ></a-asset-items>
      <a-asset-items id="base_link_left" src="base_link_left.gltf" ></a-asset-items>
      <a-asset-items id="base_link_right" src="base_link_right.gltf" ></a-asset-items>
      <a-asset-items id="link1_left" src="link1_left.gltf" ></a-asset-items>
      <a-asset-items id="link1_right" src="link1_right.gltf" ></a-asset-items>
      <a-asset-items id="link2_left" src="link2_left.gltf" ></a-asset-items>
      <a-asset-items id="link2_right" src="link2_right.gltf" ></a-asset-items>
      <a-asset-items id="link3_left" src="link3_left.gltf" ></a-asset-items>
      <a-asset-items id="link3_right" src="link3_right.gltf" ></a-asset-items>
      <a-asset-items id="link4_left" src="link4_left.gltf" ></a-asset-items>
      <a-asset-items id="link4_right" src="link4_right.gltf" ></a-asset-items>
      <a-asset-items id="link5_left" src="link5_left.gltf" ></a-asset-items>
      <a-asset-items id="link5_right" src="link5_right.gltf" ></a-asset-items>
      <a-asset-items id="link6_left" src="link6_left.gltf" ></a-asset-items>
      <a-asset-items id="link6_right" src="link6_right.gltf" ></a-asset-items>
      <a-asset-items id="gripper_base_left" src="gripper_base_left.gltf" ></a-asset-items>
      <a-asset-items id="gripper_base_right" src="gripper_base_right.gltf" ></a-asset-items>
      <a-asset-items id="finger_inner_left" src="finger_inner_left.gltf" ></a-asset-items>
      <a-asset-items id="finger_inner_right" src="finger_inner_right.gltf" ></a-asset-items>
      <a-asset-items id="finger_outer_left" src="finger_outer_left.gltf" ></a-asset-items>
      <a-asset-items id="finger_outer_right" src="finger_outer_right.gltf" ></a-asset-items>
    </a-assets>
  )
}

const Robot_data = (props)=>{
  const {visible=true} = props
  return (<>{visible?
      <a-entity gltf-model="#body_link0" obj-id="body_link0">
        <a-entity gltf-model="#base_link_left" obj-id="base_link_left">
          <a-entity gltf-model="#link1_left" obj-id="link1_left">
            <a-entity gltf-model="#link2_left" obj-id="link2_left">
              <a-entity gltf-model="#link3_left" obj-id="link3_left">
                <a-entity gltf-model="#link4_left" obj-id="link4_left">
                  <a-entity gltf-model="#link5_left" obj-id="link5_left">
                    <a-entity gltf-model="#link6_left" obj-id="link6_left">
                      <a-entity gltf-model="#gripper_base_left" obj-id="gripper_base_left">
                        <a-entity gltf-model="#finger_inner_left" obj-id="finger_inner_left">
                        </a-entity>
                        <a-entity gltf-model="#finger_outer_left" obj-id="finger_outer_left">
                        </a-entity>
                      </a-entity>
                    </a-entity>
                  </a-entity>
                </a-entity>
              </a-entity>
            </a-entity>
          </a-entity>
        </a-entity>
        <a-entity gltf-model="#base_link_right" obj-id="base_link_right">
          <a-entity gltf-model="#link1_right" obj-id="link1_right">
            <a-entity gltf-model="#link2_right" obj-id="link2_right">
              <a-entity gltf-model="#link3_right" obj-id="link3_right">
                <a-entity gltf-model="#link4_right" obj-id="link4_right">
                  <a-entity gltf-model="#link5_right" obj-id="link5_right">
                    <a-entity gltf-model="#link6_right" obj-id="link6_right">
                      <a-entity gltf-model="#gripper_base_right" obj-id="gripper_base_right">
                        <a-entity gltf-model="#finger_inner_right" obj-id="finger_inner_right">
                        </a-entity>
                        <a-entity gltf-model="#finger_outer_right" obj-id="finger_outer_right">
                        </a-entity>
                      </a-entity>
                    </a-entity>
                  </a-entity>
                </a-entity>
              </a-entity>
            </a-entity>
          </a-entity>
        </a-entity>
      </a-entity>
    :null}</>
  )
}

const normalize180 = (angle)=>{
  if(Math.abs(angle) === 180){
    return angle
  }
  return ((angle + 180) % 360 + 360) % 360 - 180
}
const toAngle = (radian)=>{
  return normalize180(radian*(180/Math.PI))
}
const toRadian = (angle)=>{
  return normalize180(angle)*(Math.PI/180)
}
