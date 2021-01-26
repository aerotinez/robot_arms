include <swivel_joint_bearing.scad>
include <swivel_joint_knuckle.scad>
include <pin_joint_bearing.scad>
include <pin_joint_knuckle.scad>
include <cube_link.scad>

module assembly_3(w_j2k = 100, r_j2k = 50, l_l2 = 20, w_j3b = 100, r_j3b = 50){
    rotate([90, -90, 0]){
        union(){
            //  joint 2 knuckle
            pin_joint_knuckle(w_j2k, r_j2k);
            //  link 2
            translate(v = [r_j2k + l_l2/2, 0, 0]){
                cube_link(2*r_j2k, 2*r_j2k, l_l2, 10);
            }
            //  joint 3 bearing
            translate(v = [r_j2k + l_l2 + r_j3b, 0, 0]){
                pin_joint_bearing(w_j3b, r_j3b);
            }
        }
    }
}
//  assembly_3();