include <swivel_joint_bearing.scad>
include <swivel_joint_knuckle.scad>
include <pin_joint_bearing.scad>
include <pin_joint_knuckle.scad>
include <cube_link.scad>

module assembly_5(h_j4k = 20, r_j4k = 50, l_l4 = 10, w_j5b = 100, r_j5b = 50){
    rotate([90, -90, 0]){
        union(){
            //  joint 4 knuckle
            rotate([0, 90, 0]){
                swivel_joint_knuckle(h_j4k, r_j4k, 2);
            }
            //  link 4
            translate(v = [h_j4k + l_l4/2, 0, 0]){
                cube_link(2*r_j4k, 2*r_j4k, l_l4, 0);
            }
            //  joint 5 bearing
            translate(v = [h_j4k + l_l4 + r_j5b, 0, 0]){
                pin_joint_bearing(w_j5b, r_j5b);
            }
        }
    }
}
assembly_5();