// IU8-34, Milchenko Ivan, 14 Variant
// F = cx -> max
// Ax <= b

#include "Symplex.h"

int main()
{
    std::string path = "/Users/ivanmilchenko/MO_lab_02/nikinput.txt";
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("Unable to open file " + path);

    Symplex problem(input);

    std::ifstream input1(path);
    Symplex dual_problem(input1);

    std::cout << "THIS IS YOUR PROBLEM IN SYMPLEX TABLE:\n";
    problem.print();

    size_t iterations = 0;

    while (problem.is_optimal())
    {
        ++iterations;
        std::cout << "THE " << iterations << " STEP:\n";
        problem.do_step();
        problem.print();
    }

    std::cout << "THE SOLUTION IS " << problem.get_solution() << std::endl;
    problem.do_examination();
    std::cout << std::endl;

    // DUAL PROBLEM
    std::cout << "DUAL PROBLEM:\n";
    dual_problem.make_dual();
    dual_problem.print();
    iterations = 0;

    while(dual_problem.do_step())
    {
        ++iterations;
        std::cout << "THE " << iterations << " STEP:\n";
        dual_problem.print();
    }

    std::cout << "THE SOLUTION IS " << -dual_problem.get_solution() << std::endl;
    dual_problem.do_dual_examination();
    std::cout << std::endl;

    return 0;
}
