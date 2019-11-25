#ifndef  TF_COORDINATES_HPP
#define  TF_COORDINATES_HPP


struct  tfinfo {
    float  x;
    float  y;
    float  z;
    float  roll;
    float  pitch;
    float  yaw;
};

struct tfinfo TF_LR1_LR2_LiveR1 = {-0.263958, 0.149845, 3.04204, 2.65819, 0.101904, 0.0475162};
//struct tfinfo TF_LR1_LR2_LiveR2 = {-0.902439, 1.06792, 7.89594, -2.47327, -0.135465, 3.07447};
struct tfinfo TF_LR1_LR2_LiveR2 = {-0.842647, 1.11238,6.74267, -2.53632, -0.21072, 3.0479};
struct tfinfo TF_XF_Guest_Guest = {-2.12693, -0.817213, 2.50755, 2.42885, 0.12389, 0.553447};
struct tfinfo TF_XF_Guest_XF = {0.474756, 0.667762, 10.5884, 2.31277, -0.706958, 0.432867};
struct tfinfo TF_XF_LR2_LiveR2 = {0.617855, 0.694411, 10.1231, 2.27825, -0.390063, 0.0369801};
struct tfinfo TF_XF_LR2_XF = {0.597557, 1.00005, 4.41286, -1.94835, -0.609637, -1.90576};
struct tfinfo TF_XF_LR3_LiveR3 = {-1.55544, -0.947621, 6.00881, -2.71076, 0.422284, 1.80814};
struct tfinfo TF_XF_LR3_XF = {-0.555036, 0.753251, 8.51888, -2.31808, -0.316065, -1.3607};


#endif