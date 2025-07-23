#include "blast_rush.h"

using namespace blast;

void print(int a, bool endl = true) {
    std::cout << a;
    if (endl) {
        std::cout << std::endl;
    }
}

void print(float a, bool endl = true) {
    std::cout << a;
    if (endl) {
        std::cout << std::endl;
    }
}

template <typename T>
void print(std::vector<T> a, bool endl = true) {
    if (a.size() == 0) {
        std::cout << "error: impossible to print vector with size 0" << std::endl;
        return;
    }
    std::cout << "[ ";
    for (int i = 0; i < a.size()-1; i++) {
        print(a[i], false);
        std::cout << ", ";
    }
    print(a.back(), false);
    std::cout << " ]";
    if (endl) {
        std::cout << std::endl;
    }
}

std::vector<blast::Matrix> get_Link6_demo1_tasks_simple() {
    blast::Array pos_wb1 = blast::deg2rad({51.851436614990234,
                             -13.578636169433594,
                             107.87167358398438,
                             3.6194305419921875,
                             33.133209228515625,
                             51.21833801269531});
    blast::Array pos_wb3 = blast::deg2rad({53.879573822021484,
                             -6.246559143066406,
                             121.92112731933594,
                             5.29833984375,
                             34.81269836425781,
                             45.617919921875});
    blast::Array pos_wb4 = blast::deg2rad({62.90712356567383,
                             -2.7591018676757812,
                             119.23771667480469,
                             8.18115234375,
                             28.954452514648438,
                             47.103912353515625});

    blast::Array pos_bb2 = blast::deg2rad({27.443552017211914,
                             -54.93880081176758,
                             28.312477111816406,
                             1.807525634765625,
                             -4.7853546142578125,
                             29.731155395507812});
    blast::Array pos_bb5 = blast::deg2rad({32.459312438964844,
                             -41.956546783447266,
                             48.75336837768555,
                             4.1028289794921875,
                             1.5039520263671875,
                             15.067718505859375});
    blast::Array pos_bb6 = blast::deg2rad({29.169939041137695,
                             -44.1719856262207,
                             55.125301361083984,
                             0.5225830078125,
                             12.9627685546875,
                             25.127288818359375});

    blast::Array pos_w1_10cm = blast::deg2rad({-40.445762634277344,
                                 -26.876392364501953,
                                 83.60868835449219,
                                 1.49383544921875,
                                 19.951095581054688,
                                 -42.22943115234375});

    blast::Array pos_b2_10cm = blast::deg2rad({-24.826316833496094,
                                 -9.07125473022461,
                                 110.22100830078125,
                                 1.2542877197265625,
                                 30.40911865234375,
                                 -27.223480224609375});

    blast::Array pos_w3_10cm = blast::deg2rad({-33.93860626220703,
                                 -35.56209945678711,
                                 70.89104461669922,
                                 -0.6509857177734375,
                                 16.881134033203125,
                                 -31.8890380859375});

    blast::Array pos_w4_10cm = blast::deg2rad({-19.595504760742188,
                                 -19.332134246826172,
                                 95.92303466796875,
                                 -3.042205810546875,
                                 23.505020141601562,
                                 -17.61700439453125});

    blast::Array pos_b5_10cm = blast::deg2rad({-31.708511352539062,
                                 -46.73225784301758,
                                 46.94545364379883,
                                 1.8437347412109375,
                                 3.8740386962890625,
                                 -30.906951904296875});

    blast::Array pos_b6_10cm = blast::deg2rad({-17.104469299316406,
                                 -30.512935638427734,
                                 77.69818115234375,
                                 0.307220458984375,
                                 17.362060546875,
                                 -12.498809814453125});

    std::vector<blast::Matrix> tasks;
    blast::Matrix task_1(6, 6);
    blast::Matrix task_2(6, 6);
    blast::Matrix task_3(6, 6);
    blast::Matrix task_4(6, 6);
    blast::Matrix task_5(6, 6);
    blast::Matrix task_6(6, 6);

    for (int i = 0; i < 6; i++) {
        // Bloc 1
        task_1(i, 0) = pos_w1_10cm[i];
        task_1(i, 3) = pos_wb1[i];

        // Bloc 2
        task_2(i, 0) = pos_b2_10cm[i];
        task_2(i, 3) = pos_bb2[i];

        // Bloc 3
        task_3(i, 0) = pos_w3_10cm[i];
        task_3(i, 3) = pos_wb3[i];

        // Bloc 4
        task_4(i, 0) = pos_w4_10cm[i];
        task_4(i, 3) = pos_wb4[i];

        // Bloc 5
        task_5(i, 0) = pos_b5_10cm[i];
        task_5(i, 3) = pos_bb5[i];

        // Bloc 6
        task_6(i, 0) = pos_b6_10cm[i];
        task_6(i, 3) = pos_bb6[i];
    }

    // Sequence tasks
    tasks.push_back(task_1);
    tasks.push_back(task_2);
    tasks.push_back(task_3);
    tasks.push_back(task_4);
    tasks.push_back(task_5);
    tasks.push_back(task_6);

    return tasks;
}
