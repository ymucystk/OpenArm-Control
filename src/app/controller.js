"use client";
import * as React from 'react'
import "./controller.css";

export default function Controller(props) {
  const {vrMode} = props
  const {body_link0_rotate, base_link_left_rotate, base_link_right_rotate} = props
  const {link1_left_rotate, link1_right_rotate, link2_left_rotate, link2_right_rotate, link3_left_rotate, link3_right_rotate} = props
  const {link4_left_rotate, link4_right_rotate, link5_left_rotate, link5_right_rotate, link6_left_rotate, link6_right_rotate} = props
  const {gripper_base_left_rotate, gripper_base_right_rotate} = props
  const {finger_inner_left_rotate, finger_inner_right_rotate, finger_outer_left_rotate, finger_outer_right_rotate} = props
  const {c_pos_x, c_pos_y, c_pos_z} = props
  const {c_deg_x, c_deg_y, c_deg_z} = props
  
  const set_body_link0_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_body_link0_rotate(value)
  }

  const set_base_link_right_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_base_link_right_rotate(value)
  }

  const set_base_link_left_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_base_link_left_rotate(value)
  }

  const set_link1_right_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link1_right_rotate(value)
  }

  const set_link1_left_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link1_left_rotate(value)
  }

  const set_link2_right_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link2_right_rotate(value)
  }

  const set_link2_left_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link2_left_rotate(value)
  }

  const set_link3_right_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link3_right_rotate(value)
  }

  const set_link3_left_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link3_left_rotate(value)
  }

  const set_link4_right_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link4_right_rotate(value)
  }

  const set_link4_left_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link4_left_rotate(value)
  }

  const set_link5_right_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link5_right_rotate(value)
  }

  const set_link5_left_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link5_left_rotate(value)
  }

  const set_link6_right_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link6_right_rotate(value)
  }

  const set_link6_left_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_link6_left_rotate(value)
  }

  const set_gripper_base_right_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_gripper_base_right_rotate(value)
  }

  const set_gripper_base_left_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_gripper_base_left_rotate(value)
  }

  const set_finger_inner_right_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_finger_inner_right_rotate(value)
  }

  const set_finger_inner_left_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_finger_inner_left_rotate(value)
  }

  const set_finger_outer_right_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_finger_outer_right_rotate(value)
  }

  const set_finger_outer_left_rotate = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    if(Math.abs(value)===180){
      value = value * -1
    }
    props.set_finger_outer_left_rotate(value)
  }

  const set_c_pos_x = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    props.set_c_pos_x(value)
  }

  const set_c_pos_y = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    props.set_c_pos_y(value)
  }

  const set_c_pos_z = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    props.set_c_pos_z(value)
  }

  const set_c_deg_x = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    props.set_c_deg_x(value)
  }

  const set_c_deg_y = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    props.set_c_deg_y(value)
  }

  const set_c_deg_z = (e)=>{
    let value = Number.parseFloat(e.target.value || 0)
    props.set_c_deg_z(value)
  }

  return (
    <>
      <div className="controller" style={{display: vrMode ? 'none' : undefined}}>
        <div className="row mb-2">
          <div className="col-md-4"><label htmlFor="body_rotate_number" className="form-label"><span className="form-control-plaintext">body</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="body_rotate_number" value={body_link0_rotate} onChange={set_body_link0_rotate} min={-180} max={180}/></div>
        </div>
        <div className="row mb-2">
          <div className="col-md-4"><label htmlFor="base_link_left_rotate" className="form-label"><span className="form-control-plaintext">LEFT</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="base_link_left_rotate" value={base_link_left_rotate} onChange={set_base_link_left_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link1_left_rotate" className="form-label"><span className="form-control-plaintext">LINK1</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link1_left_rotate" value={link1_left_rotate} onChange={set_link1_left_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link2_left_rotate" className="form-label"><span className="form-control-plaintext">LINK2</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link2_left_rotate" value={link2_left_rotate} onChange={set_link2_left_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link3_left_rotate" className="form-label"><span className="form-control-plaintext">LINK3</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link3_left_rotate" value={link3_left_rotate} onChange={set_link3_left_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link4_left_rotate" className="form-label"><span className="form-control-plaintext">LINK4</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link4_left_rotate" value={link4_left_rotate} onChange={set_link4_left_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link5_left_rotate" className="form-label"><span className="form-control-plaintext">LINK5</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link5_left_rotate" value={link5_left_rotate} onChange={set_link5_left_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link6_left_rotate" className="form-label"><span className="form-control-plaintext">LINK6</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link6_left_rotate" value={link6_left_rotate} onChange={set_link6_left_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="gripper_base_left_rotate" className="form-label"><span className="form-control-plaintext">GRIP</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="gripper_base_left_rotate" value={gripper_base_left_rotate} onChange={set_gripper_base_left_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="finger_inner_left_rotate" className="form-label"><span className="form-control-plaintext">FINGER1</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="finger_inner_left_rotate" value={finger_inner_left_rotate} onChange={set_finger_inner_left_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="finger_outer_left_rotate" className="form-label"><span className="form-control-plaintext">FINGER2</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="finger_outer_left_rotate" value={finger_outer_left_rotate} onChange={set_finger_outer_left_rotate} min={-180} max={180}/></div>
        </div>
        <div className="row mb-2">
          <div className="col-md-4"><label htmlFor="base_link_right_rotate" className="form-label"><span className="form-control-plaintext">RIGHT</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="base_link_right_rotate" value={base_link_right_rotate} onChange={set_base_link_right_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link1_right_rotate" className="form-label"><span className="form-control-plaintext">LINK1</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link1_right_rotate" value={link1_right_rotate} onChange={set_link1_right_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link2_right_rotate" className="form-label"><span className="form-control-plaintext">LINK2</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link2_right_rotate" value={link2_right_rotate} onChange={set_link2_right_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link3_right_rotate" className="form-label"><span className="form-control-plaintext">LINK3</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link3_right_rotate" value={link3_right_rotate} onChange={set_link3_right_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link4_right_rotate" className="form-label"><span className="form-control-plaintext">LINK4</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link4_right_rotate" value={link4_right_rotate} onChange={set_link4_right_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link5_right_rotate" className="form-label"><span className="form-control-plaintext">LINK5</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link5_right_rotate" value={link5_right_rotate} onChange={set_link5_right_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="link6_right_rotate" className="form-label"><span className="form-control-plaintext">LINK6</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="link6_right_rotate" value={link6_right_rotate} onChange={set_link6_right_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="gripper_base_right_rotate" className="form-label"><span className="form-control-plaintext">GRIP</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="gripper_base_right_rotate" value={gripper_base_right_rotate} onChange={set_gripper_base_right_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="finger_inner_right_rotate" className="form-label"><span className="form-control-plaintext">FINGER1</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="finger_inner_right_rotate" value={finger_inner_right_rotate} onChange={set_finger_inner_right_rotate} min={-180} max={180}/></div>
          <div className="col-md-4"><label htmlFor="finger_outer_right_rotate" className="form-label"><span className="form-control-plaintext">FINGER2</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="finger_outer_right_rotate" value={finger_outer_right_rotate} onChange={set_finger_outer_right_rotate} min={-180} max={180}/></div>
        </div>

      </div>
      <div className="camera-controller" style={{display: vrMode ? 'none' : undefined}}>
        <span>CAMERA</span>
        <div className="row mb-2">
          <div className="col-md-4"><label htmlFor="c_pos_x_number" className="form-label"><span className="form-control-plaintext">pos X</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_pos_x_number" value={c_pos_x} onChange={set_c_pos_x} step={0.01}/></div>
          <div className="col-md-4"><label htmlFor="c_pos_y_number" className="form-label"><span className="form-control-plaintext">pos Y</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_pos_y_number" value={c_pos_y} onChange={set_c_pos_y} step={0.01}/></div>
          <div className="col-md-4"><label htmlFor="c_pos_z_number" className="form-label"><span className="form-control-plaintext">pos Z</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_pos_z_number" value={c_pos_z} onChange={set_c_pos_z} step={0.01}/></div>
        </div>
        <div className="row mb-2">
          <div className="col-md-4"><label htmlFor="c_deg_x_number" className="form-label"><span className="form-control-plaintext">deg X</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_deg_x_number" value={c_deg_x} onChange={set_c_deg_x} step={0.1}/></div>
          <div className="col-md-4"><label htmlFor="c_deg_y_number" className="form-label"><span className="form-control-plaintext">deg Y</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_deg_y_number" value={c_deg_y} onChange={set_c_deg_y} step={0.1}/></div>
          <div className="col-md-4"><label htmlFor="c_deg_z_number" className="form-label"><span className="form-control-plaintext">deg Z</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_deg_z_number" value={c_deg_z} onChange={set_c_deg_z} step={0.1}/></div>
        </div>
      </div>
    </>
    )
}
