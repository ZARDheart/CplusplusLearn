#include "workerManager.h"

int main()
{
    WorkerManager wm;
    int choice = 0;
    while (true)
    {
        wm.Show_Menu();
        cout << endl
             << "Please input your choice:" << endl;
        cin >> choice;
        cin.get();
        switch (choice)
        {
        case 0:
            wm.ExitSystem();
            break;
        case 1:
            wm.Show_Worker();
            cout << "按回车键继续" << endl;
            cin.get();
            break;
        case 2:
            wm.Add_Worker();
            cout << "按回车键继续" << endl;
            cin.get();
            break;
        case 3:
            wm.Del_Worker();
            cout << "按回车键继续" << endl;
            cin.get();
            break;
        case 4:
            wm.Update_Worker();
            cout << "按回车键继续" << endl;
            cin.get();
            break;
        case 5:
            wm.Search_Worker();
            cout << "按回车键继续" << endl;
            cin.get();
            break;
        case 6:
            wm.Sort_Worker();
            cout << "按回车键继续" << endl;
            cin.get();
            break;
        case 7:
            wm.Clear_File();
            cout << "按回车键继续" << endl;
            cin.get();
            break;
        default:
            system("cls");
            break;
        }
    }

    return 0;
}
