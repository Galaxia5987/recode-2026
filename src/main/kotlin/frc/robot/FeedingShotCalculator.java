package frc.robot;

public class FeedingShotCalculator {

public static double calculateFeedingVelocity(double distance, double vx, double vy) {
return
        1.66391366 +
        (1.50399158) * distance +
        (-1.01758214) * vx +
        (-0.00649510) * vy +
        (-0.09576736) * Math.pow(distance, 2.0) +
        (0.01570136) * distance*vx +
        (0.00353390) * distance*vy +
        (0.08609519) * Math.pow(vx, 2.0) +
        (-0.00194281) * vx*vy +
        (0.25694915) * Math.pow(vy, 2.0) +
        (0.00528176) * Math.pow(distance, 3.0) +
        (-0.00175959) * Math.pow(distance, 2.0)*vx +
        (-0.00048673) * Math.pow(distance, 2.0)*vy +
        (-0.00761884) * distance*Math.pow(vx, 2.0) +
        (0.00058480) * distance*vx*vy +
        (-0.03245550) * distance*Math.pow(vy, 2.0) +
        (0.00057476) * Math.pow(vx, 3.0) +
        (-0.00012919) * Math.pow(vx, 2.0)*vy +
        (0.01668421) * vx*Math.pow(vy, 2.0) +
        (-0.00003216) * Math.pow(vy, 3.0) +
        (-0.00011314) * Math.pow(distance, 4.0) +
        (0.00004312) * Math.pow(distance, 3.0)*vx +
        (0.00001889) * Math.pow(distance, 3.0)*vy +
        (0.00030293) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0) +
        (-0.00003301) * Math.pow(distance, 2.0)*vx*vy +
        (0.00132043) * Math.pow(distance, 2.0)*Math.pow(vy, 2.0) +
        (-0.00006013) * distance*Math.pow(vx, 3.0) +
        (0.00000991) * distance*Math.pow(vx, 2.0)*vy +
        (-0.00138796) * distance*vx*Math.pow(vy, 2.0) +
        (0.00000922) * distance*Math.pow(vy, 3.0) +
        (-0.00023405) * Math.pow(vx, 4.0) +
        (0.00000992) * Math.pow(vx, 3.0)*vy +
        (-0.00016408) * Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-0.00003235) * vx*Math.pow(vy, 3.0) +
        (0.00070229) * Math.pow(vy, 4.0)
        ;
}




public static double calculateFeedingPitch() {
return
        55.00000000
        ;
}




public static double calculateFeedingYaw(double distance, double vx, double vy) {
return
        0.11145617 +
        (-0.02090865) * distance +
        (0.04091180) * vx +
        (-18.15993212) * vy +
        (-0.03191538) * Math.pow(distance, 2.0) +
        (0.01320683) * distance*vx +
        (-9.52461950) * distance*vy +
        (0.00778629) * Math.pow(vx, 2.0) +
        (-2.97815804) * vx*vy +
        (0.02114397) * Math.pow(vy, 2.0) +
        (0.02117662) * Math.pow(distance, 3.0) +
        (-0.02784196) * Math.pow(distance, 2.0)*vx +
        (6.94052112) * Math.pow(distance, 2.0)*vy +
        (-0.00123995) * distance*Math.pow(vx, 2.0) +
        (-2.46026528) * distance*vx*vy +
        (-0.24078404) * distance*Math.pow(vy, 2.0) +
        (0.02495367) * Math.pow(vx, 3.0) +
        (-5.91885170) * Math.pow(vx, 2.0)*vy +
        (0.03172780) * vx*Math.pow(vy, 2.0) +
        (-9.77094145) * Math.pow(vy, 3.0) +
        (-0.00594909) * Math.pow(distance, 4.0) +
        (0.01152420) * Math.pow(distance, 3.0)*vx +
        (-1.85923667) * Math.pow(distance, 3.0)*vy +
        (0.00406462) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0) +
        (1.53602611) * Math.pow(distance, 2.0)*vx*vy +
        (0.12511078) * Math.pow(distance, 2.0)*Math.pow(vy, 2.0) +
        (-0.02838533) * distance*Math.pow(vx, 3.0) +
        (2.96578127) * distance*Math.pow(vx, 2.0)*vy +
        (-0.05648528) * distance*vx*Math.pow(vy, 2.0) +
        (5.04287140) * distance*Math.pow(vy, 3.0) +
        (-0.02804220) * Math.pow(vx, 4.0) +
        (-2.25965477) * Math.pow(vx, 3.0)*vy +
        (-0.00587410) * Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-1.97604556) * vx*Math.pow(vy, 3.0) +
        (-0.07424556) * Math.pow(vy, 4.0) +
        (0.00090029) * Math.pow(distance, 5.0) +
        (-0.00221794) * Math.pow(distance, 4.0)*vx +
        (0.26105266) * Math.pow(distance, 4.0)*vy +
        (-0.00137763) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0) +
        (-0.33183124) * Math.pow(distance, 3.0)*vx*vy +
        (-0.02721101) * Math.pow(distance, 3.0)*Math.pow(vy, 2.0) +
        (0.01064727) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0) +
        (-0.62798310) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*vy +
        (0.01983608) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 2.0) +
        (-1.13551995) * Math.pow(distance, 2.0)*Math.pow(vy, 3.0) +
        (-0.00191823) * distance*Math.pow(vx, 4.0) +
        (0.97393713) * distance*Math.pow(vx, 3.0)*vy +
        (0.01259350) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (0.83420998) * distance*vx*Math.pow(vy, 3.0) +
        (0.07408163) * distance*Math.pow(vy, 4.0) +
        (-0.00598346) * Math.pow(vx, 5.0) +
        (0.44746641) * Math.pow(vx, 4.0)*vy +
        (-0.00484358) * Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (0.82254452) * Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (-0.02725193) * vx*Math.pow(vy, 4.0) +
        (2.34937278) * Math.pow(vy, 5.0) +
        (-0.00007570) * Math.pow(distance, 6.0) +
        (0.00022261) * Math.pow(distance, 5.0)*vx +
        (-0.02029935) * Math.pow(distance, 5.0)*vy +
        (0.00017705) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0) +
        (0.03474144) * Math.pow(distance, 4.0)*vx*vy +
        (0.00302281) * Math.pow(distance, 4.0)*Math.pow(vy, 2.0) +
        (-0.00164718) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0) +
        (0.06679178) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*vy +
        (-0.00315657) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 2.0) +
        (0.13034227) * Math.pow(distance, 3.0)*Math.pow(vy, 3.0) +
        (0.00128398) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0) +
        (-0.16626865) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*vy +
        (-0.00363505) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-0.14337065) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 3.0) +
        (-0.01960426) * Math.pow(distance, 2.0)*Math.pow(vy, 4.0) +
        (0.00028285) * distance*Math.pow(vx, 5.0) +
        (-0.10543578) * distance*Math.pow(vx, 4.0)*vy +
        (0.00574799) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (-0.19756551) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (0.01689199) * distance*vx*Math.pow(vy, 4.0) +
        (-0.57346949) * distance*Math.pow(vy, 5.0) +
        (0.01186857) * Math.pow(vx, 6.0) +
        (0.21037912) * Math.pow(vx, 5.0)*vy +
        (0.00508693) * Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (0.24598243) * Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (0.00716200) * Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (0.25807213) * vx*Math.pow(vy, 5.0) +
        (0.04639451) * Math.pow(vy, 6.0) +
        (0.00000332) * Math.pow(distance, 7.0) +
        (-0.00001124) * Math.pow(distance, 6.0)*vx +
        (0.00082712) * Math.pow(distance, 6.0)*vy +
        (-0.00001016) * Math.pow(distance, 5.0)*Math.pow(vx, 2.0) +
        (-0.00178118) * Math.pow(distance, 5.0)*vx*vy +
        (-0.00016812) * Math.pow(distance, 5.0)*Math.pow(vy, 2.0) +
        (0.00011249) * Math.pow(distance, 4.0)*Math.pow(vx, 3.0) +
        (-0.00351900) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0)*vy +
        (0.00023640) * Math.pow(distance, 4.0)*vx*Math.pow(vy, 2.0) +
        (-0.00744829) * Math.pow(distance, 4.0)*Math.pow(vy, 3.0) +
        (-0.00011539) * Math.pow(distance, 3.0)*Math.pow(vx, 4.0) +
        (0.01249693) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0)*vy +
        (0.00034859) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (0.01104839) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 3.0) +
        (0.00187552) * Math.pow(distance, 3.0)*Math.pow(vy, 4.0) +
        (0.00001207) * Math.pow(distance, 2.0)*Math.pow(vx, 5.0) +
        (0.00974759) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0)*vy +
        (-0.00090190) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (0.01857238) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (-0.00237913) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 4.0) +
        (0.05457678) * Math.pow(distance, 2.0)*Math.pow(vy, 5.0) +
        (-0.00141646) * distance*Math.pow(vx, 6.0) +
        (-0.03356369) * distance*Math.pow(vx, 5.0)*vy +
        (-0.00083143) * distance*Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (-0.03942181) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (-0.00069296) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (-0.04406434) * distance*vx*Math.pow(vy, 5.0) +
        (-0.00730773) * distance*Math.pow(vy, 6.0) +
        (0.00188682) * Math.pow(vx, 7.0) +
        (-0.02345942) * Math.pow(vx, 6.0)*vy +
        (0.00067601) * Math.pow(vx, 5.0)*Math.pow(vy, 2.0) +
        (-0.04492811) * Math.pow(vx, 4.0)*Math.pow(vy, 3.0) +
        (0.00075628) * Math.pow(vx, 3.0)*Math.pow(vy, 4.0) +
        (-0.07069481) * Math.pow(vx, 2.0)*Math.pow(vy, 5.0) +
        (0.00237020) * vx*Math.pow(vy, 6.0) +
        (-0.19471310) * Math.pow(vy, 7.0) +
        (-0.00000006) * Math.pow(distance, 8.0) +
        (0.00000023) * Math.pow(distance, 7.0)*vx +
        (-0.00001378) * Math.pow(distance, 7.0)*vy +
        (0.00000022) * Math.pow(distance, 6.0)*Math.pow(vx, 2.0) +
        (0.00003583) * Math.pow(distance, 6.0)*vx*vy +
        (0.00000371) * Math.pow(distance, 6.0)*Math.pow(vy, 2.0) +
        (-0.00000287) * Math.pow(distance, 5.0)*Math.pow(vx, 3.0) +
        (0.00007332) * Math.pow(distance, 5.0)*Math.pow(vx, 2.0)*vy +
        (-0.00000669) * Math.pow(distance, 5.0)*vx*Math.pow(vy, 2.0) +
        (0.00016847) * Math.pow(distance, 5.0)*Math.pow(vy, 3.0) +
        (0.00000265) * Math.pow(distance, 4.0)*Math.pow(vx, 4.0) +
        (-0.00034735) * Math.pow(distance, 4.0)*Math.pow(vx, 3.0)*vy +
        (-0.00001197) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-0.00031734) * Math.pow(distance, 4.0)*vx*Math.pow(vy, 3.0) +
        (-0.00006512) * Math.pow(distance, 4.0)*Math.pow(vy, 4.0) +
        (0.00000440) * Math.pow(distance, 3.0)*Math.pow(vx, 5.0) +
        (-0.00032969) * Math.pow(distance, 3.0)*Math.pow(vx, 4.0)*vy +
        (0.00004579) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (-0.00065460) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (0.00010709) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 4.0) +
        (-0.00195032) * Math.pow(distance, 3.0)*Math.pow(vy, 5.0) +
        (0.00007162) * Math.pow(distance, 2.0)*Math.pow(vx, 6.0) +
        (0.00180562) * Math.pow(distance, 2.0)*Math.pow(vx, 5.0)*vy +
        (0.00006921) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (0.00215071) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (0.00012277) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (0.00232248) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 5.0) +
        (0.00070759) * Math.pow(distance, 2.0)*Math.pow(vy, 6.0) +
        (-0.00024456) * distance*Math.pow(vx, 7.0) +
        (0.00182930) * distance*Math.pow(vx, 6.0)*vy +
        (-0.00023027) * distance*Math.pow(vx, 5.0)*Math.pow(vy, 2.0) +
        (0.00381885) * distance*Math.pow(vx, 4.0)*Math.pow(vy, 3.0) +
        (-0.00035952) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 4.0) +
        (0.00648515) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 5.0) +
        (-0.00055463) * distance*vx*Math.pow(vy, 6.0) +
        (0.01795374) * distance*Math.pow(vy, 7.0) +
        (-0.00074320) * Math.pow(vx, 8.0) +
        (-0.00889252) * Math.pow(vx, 7.0)*vy +
        (-0.00050320) * Math.pow(vx, 6.0)*Math.pow(vy, 2.0) +
        (-0.01175635) * Math.pow(vx, 5.0)*Math.pow(vy, 3.0) +
        (-0.00089069) * Math.pow(vx, 4.0)*Math.pow(vy, 4.0) +
        (-0.01133526) * Math.pow(vx, 3.0)*Math.pow(vy, 5.0) +
        (-0.00239771) * Math.pow(vx, 2.0)*Math.pow(vy, 6.0) +
        (-0.00894433) * vx*Math.pow(vy, 7.0) +
        (-0.00767672) * Math.pow(vy, 8.0)
        ;
}

}
