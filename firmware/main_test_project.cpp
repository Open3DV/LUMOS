#include "base_projector.h"
#include "ainstec_projector.h"

int main()
{
    char* info = "\
    Ainstec光机测试程序：\n\
    1. 设置曝光时间（单位：us）；\n\
    2. 设置投影亮度（范围：[0, 1023]）；\n\
    3. 设置投影模式；\n\
    4. 投影一组条纹；\n";

    AinstecProjector proejctorTest;
    proejctorTest.init();

    while(1)
    {
        std::cout << info;
        int choose;
        std::cin >> choose;

        switch (choose)
        {
        case 1:
            int exposure;
            std::cout << "请输入曝光时间（单位：us）\n";
            std::cin >> exposure;
            if (proejctorTest.setProjectorExposure(exposure))
            {
                std::cout << "运行成功！" << std::endl;
            }
            break;
        case 2:
            int current;
            std::cout << "请输入投影亮度：\n" << std::endl;
            std::cin >> current;
            if (proejctorTest.setProjectorCurrent(current))
            {
                std::cout << "运行成功！" << std::endl;
            }
            break;        
        case 3:
            int mode;
            std::cout << "请输入选择的投影模式：" << std::endl;
            std::cin >> mode;
            if (proejctorTest.setProjectorWorkingMode(mode))
            {
                std::cout << "运行成功！" << std::endl;
            }
            break;
        case 4:
            if (proejctorTest.project())
            {
                std::cout << "运行成功！" << std::endl;
            }
            break;
        default:
            break;
        }

    }
}