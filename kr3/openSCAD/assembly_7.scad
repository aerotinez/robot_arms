include <swivel_joint_bearing.scad>
include <swivel_joint_knuckle.scad>
include <pin_joint_bearing.scad>
include <pin_joint_knuckle.scad>
include <cube_link.scad>

module assembly_7(h_j6k = 20, r_j6k = 50, h_l6 = 20, r_l6 = 25, joint_chamfer = 2){
    rotate([90, -90, 0]){
        union(){
            //  joint 6 knuckle
            rotate([0, 90, 0]){
                swivel_joint_knuckle(h_j6k, r_j6k, joint_chamfer);
            }
            //  link6
            translate(v = [h_j6k + h_l6/2, 0, 0]){
                rotate([0, 90, 0]){
                    cylinder(h = h_l6, r = r_l6, center = true);
                }
            }
        }
    }
}
assembly_7();