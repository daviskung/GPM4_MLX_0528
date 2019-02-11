#if 1
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include "string_sample.h"

using namespace std;

extern "C" {

void string_sample(void)
{
    string password="C++";
    string typostr;

    cout << "Hello C++ World! \n" << "Typing C++ to proceed:";

    while (1)
    {
        cin >> typostr; /* Note: will crash if input much characters than str size */

        if (typostr == password)
            break;

        cout << "Wrong input:" << typostr << endl;
        cout << "Input again:";
    }

    cout << "Bingo! Password is exactly " + password + " !" << endl;
}

}
#endif
