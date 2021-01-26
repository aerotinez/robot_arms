module cube_link(breadth= 10, depth= 10, length = 20, joint_clearance = 10){
    L = length;
    jc = joint_clearance;
    l = L - jc;
    r = ((l - L)/2) + jc;
    rotate([0, 90, 0]){
        union(){
            translate(v = [0, 0, jc/2]){
                cube([breadth, depth, l], center = true);
            }
            translate(v = [0, 0, -((l/2 - r) + jc/2)]){
                cube([0.6*breadth, depth, jc], center = true);
            }
        }
    }
}

/*
//  UNCOMMENT TO TEST MODULE
translate(v = [0, 0, 0])
cube_link(10, 10, 20, 5);
*/