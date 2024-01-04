#include <fstream>
#include <unistd.h>
#include <iostream>

bool read_reboot_flag_from_file(std::string flie_path, bool& reboot_now)
{
    std::fstream fs(flie_path);

    if (!fs.is_open())
    {
        std::cout << "open_file_failed" << std::endl;
        return false;
    }

    int reboot_flag = 0;
    fs >> reboot_flag;

    reboot_now = reboot_flag == 1 ? true : false;
    fs.close();
    if (reboot_now)
    {
        std::ofstream ofs(flie_path);
        ofs << "0";
        ofs.close();
    }
    else if (reboot_flag == 2)
    {
        int wait_time = 0;
        while(wait_time < 20)
        {
            std::ifstream ifs(flie_path);
            ifs >> reboot_flag;
            if (reboot_flag != 2)
            {
                break;
            }
            else
            {
                sleep(1);
                wait_time += 1;
                std::cout << "启动计时，20s内未启动成功则重启：" << wait_time << std::endl;
            }
            ifs.close();
        }
        if (reboot_flag != 0)
        {
            std::ofstream ofs(flie_path);
            ofs << "0";
            ofs.close();
            reboot_now = true;
        }
        // 读取目标的内容，若目标的内容是2，并且持续了20s则对机器进行重启。
    }

    return true;
}

bool reboot_times_control(std::string file_path, int& reboot_times, bool add_one_time)
{
    std::ifstream ifs(file_path);
    if (ifs.is_open())
    {
        ifs >> reboot_times;
    }
    else
    {
        return false;
    }
    ifs.close();

    if (add_one_time)
    {
        std::ofstream ofs(file_path);
        if (ofs.is_open())
        {
            ofs << reboot_times + 1;
        }
        else
        {
            return false;
        }
        ofs.close();
    }
    else
    {
        return false;
    }

    return true;
}

bool reboot_times_flush(std::string file_path)
{
    std::ofstream ofs(file_path);
    if (ofs.is_open())
    {
        ofs << 0;
        ofs.close();
    }
    else
    {
        return false;
    }

    return true;
}

bool reboot_flag_enable(std::string file_path)
{
    std::ofstream ofs(file_path);

    if (ofs.is_open())
    {
        ofs << 1;
        ofs.close();
    }
    else
    {
        return false;
    }

    return true;

}

int main()
{
    while (true)
    {
        bool reboot_now = false;
        if (read_reboot_flag_from_file("./reboot_flag.txt", reboot_now))
        {
            int reboot_times = 0;
            reboot_times_control("reboot_times.txt", reboot_times, false);
            if (reboot_now && reboot_times < 5)
            {
                reboot_times_control("reboot_times.txt", reboot_times, true);
                system("sudo init 6");
            }
            else if (reboot_times >= 10)
            {
                std::cout << "重启次数超过10次，将不重启。" << std::endl;

            }
            //reboot_flag_enable("./reboot_flag.txt");

        }
        std::cout << "\n等待命令中.";
        sleep(1);
    }

    //括号内是你的linux指令
    return 0;

}

