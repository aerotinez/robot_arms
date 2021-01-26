include <assembly_1.scad>
include <assembly_2.scad>
include <assembly_3.scad>
include <assembly_4.scad>
include <assembly_5.scad>
include <assembly_6.scad>
include <assembly_7.scad>
include <drill.scad>

translate(v = [0, 0, 25]) color("Maroon") assembly_1();
translate(v = [0, 0, 50]) color("DarkOrange") assembly_2();
translate(v = [0, 0, 160]) color("Gold") assembly_3();
translate(v = [0, 0, 280]) color("Olive") assembly_4();
translate(v = [0, 0, 370]) color("ForestGreen") assembly_5();
translate(v = [0, 0, 450]) color("Teal") assembly_6();
translate(v = [0, 0, 540]) color("Purple") assembly_7();
translate(v = [0, 0, 600]) color("Silver") drill();
