include <swivel_joint_bearing.scad>
include <swivel_joint_knuckle.scad>
include <pin_joint_bearing.scad>
include <pin_joint_knuckle.scad>
include <cube_link.scad>

//  joint 1 bearing
h_j1b = 50;    //  height of joint 1 bearing
r_j1b = 100;    //  radius of joint 1 bearing
translate(v = [h_j1b/2, 0, 0]){
    rotate([0, 90, 0]){
        swivel_joint_bearing(h_j1b, r_j1b, 5);
    }
}

//  joint 1 knuckle
h_j1k = h_j1b;  //  height of joint 1 knuckle
r_j1k = r_j1b;  //  radius of joint 1 knuckle
translate(v = [h_j1k, 0, 0]){
    rotate([0, 90, 0]){
        swivel_joint_knuckle(h_j1k, r_j1k, 5);
    }
}

//  link 1
l_l1 = 10;  //  length of link 1 (x-axis)
b_l1 = 100;   //  breadth of link 1 (y-axis)
d_l1 = 100;   //  depth of link 1 (z-axis)
translate(v = [2*h_j1k + l_l1/2, 0, 0]){
    cube_link(b_l1, d_l1, l_l1, 0);
}
a12 = 2*h_j1k + l_l1;

//  joint 2 bearing
w_j2b = 100; //  width of joint 2 bearing
r_j2b = 50; //  radius of joint 2 bearing
translate(v = [a12 + r_j2b, 0, 0]){
    pin_joint_bearing(w_j2b, r_j2b);
}

//  joint 2 knuckle
w_j2k = w_j2b;  //  width of joint 2 knuckle
r_j2k = r_j2b;  //  radius of joint 2 knuckle
translate(v = [a12 + r_j2b, 0, 0]){
    pin_joint_knuckle(w_j2k, r_j2k);
}
a21 = a12 + r_j2b;

//  link 2
l_l2 = 20;  //  length of link 2 (x-axis)
b_l2 = 2*r_j2k;   //  breadth of link 2 (y-axis)
d_l2 = 2*r_j2k;   //  depth of link 2 (z-axis)
translate(v = [a21 + r_j2b + l_l2/2, 0, 0]){
    cube_link(b_l2, d_l2, l_l2, 10);
}
a22 = a21 + r_j2b + l_l2;

//  joint 3 bearing
w_j3b = 100; //  width of joint 3 bearing
r_j3b = 50; //  radius of joint 3 bearing
translate(v = [a22 + r_j3b, 0, 0]){
    pin_joint_bearing(w_j3b, r_j3b);
}

//  joint 3 knuckle
w_j3k = w_j3b;  //  width of joint 3 knuckle
r_j3k = r_j3b;  //  radius of joint 3 knuckle
translate(v = [a22 + r_j3b, 0, 0]){
    pin_joint_knuckle(w_j3k, r_j3k);
}
a31 = a22 + 2*r_j3b;

//  link 3
l_l3 = 20;  //  length of link 3 (x-axis)
b_l3 = 2*r_j3k;   //  breadth of link 3 (y-axis)
d_l3 = 2*r_j3k;   //  depth of link 3 (z-axis)
translate(v = [a31 + l_l3/2, 0, 0]){
    cube_link(b_l3, d_l3, l_l3, 10);
}
a32 = a31 + l_l3;

//  joint 4 bearing
h_j4b = 20;    //  height of joint 1 bearing
r_j4b = 50;    //  radius of joint 1 bearing
translate(v = [a32 + h_j4b/2, 0, 0]){
    rotate([0, 90, 0]){
        swivel_joint_bearing(h_j4b, r_j4b, 2);
    }
}

//  joint 4 knuckle
h_j4k = h_j4b;    //  height of joint 1 bearing
r_j4k = r_j4b;    //  radius of joint 1 bearing
translate(v = [a32 + h_j4k, 0, 0]){
    rotate([0, 90, 0]){
        swivel_joint_knuckle(h_j4k, r_j4k, 2);
    }
}
a41 = a32 + 2*h_j4k;


//  link 4
l_l4 = 10;  //  length of link 4 (x-axis)
b_l4 = 2*r_j4k;   //  breadth of link 4 (y-axis)
d_l4 = 2*r_j4k;   //  depth of link 4 (z-axis)
translate(v = [a41 + l_l4/2, 0, 0]){
    cube_link(b_l4, d_l4, l_l4, 0);
}
a42 = a41 + l_l4;

//  joint 5 bearing
w_j5b = 100; //  width of joint 5 bearing
r_j5b = 50; //  radius of joint 5 bearing
translate(v = [a42 + r_j5b, 0, 0]){
    pin_joint_bearing(w_j2b, r_j2b);
}

//  joint 5 knuckle
w_j5k = w_j5b;  //  width of joint 5 knuckle
r_j5k = r_j5b;  //  radius of joint 5 knuckle
translate(v = [a42 + r_j5b, 0, 0]){
    pin_joint_knuckle(w_j5k, r_j5k);
}
a51 = a42 + 2*r_j5b;

//  link 5
l_l5 = 20;  //  length of link 5 (x-axis)
b_l5 = 2*r_j5k;   //  breadth of link 5 (y-axis)
d_l5 = 2*r_j5k;   //  depth of link 5 (z-axis)
translate(v = [a51 + l_l5/2, 0, 0]){
    cube_link(b_l5, d_l5, l_l5, 10);
}
a52 = a51 + l_l5;

//  joint 6 bearing
h_j6b = 20;    //  height of joint 6 bearing
r_j6b = 50;    //  radius of joint 6 bearing
translate(v = [a52 + h_j6b/2, 0, 0]){
    rotate([0, 90, 0]){
        swivel_joint_bearing(h_j6b, r_j6b, 2);
    }
}
a61 = a52 + h_j6b;

//  joint 6 knuckle
h_j6k = h_j6b;    //  height of joint 1 bearing
r_j6k = r_j6b;    //  radius of joint 1 bearing
translate(v = [a61, 0, 0]){
    rotate([0, 90, 0]){
        swivel_joint_knuckle(h_j6k, r_j6k, 2);
    }
}
a62 = a61 + h_j6b;

//  link6
h_l6 = 20;  //  height of link 6
r_l6 = 25;  //  radius of link 6
translate(v = [a62 + h_l6/2, 0, 0]){
    rotate([0, 90, 0]){
        cylinder(h = h_l6, r = r_l6, center = true);
    }
}
a_t1 = a62 + h_l6;

//  tool
h_t = 40;
r_t = 25;
translate(v = [a_t1 + h_t/2, 0, 0]){
    rotate([0, 90, 0]){
        cylinder(h = h_t, r1 = r_t, r2 = 2, center = true);
    }
}