#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <iostream>
#include <exception>

class Exception : public std::exception
{
public:
    Exception(const char* errorMsg):errorMsg_(errorMsg) {};

    virtual const char* what() const throw()
    {
        return errorMsg_;
    }

private:
    const char* errorMsg_;
};

#endif
