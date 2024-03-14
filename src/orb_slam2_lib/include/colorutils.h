#ifndef CALIB_UTILS_H
#define CALIB_UTILS_H

#include <iostream>

const std::string RESET = "\033[0m";
const std::string BLACK = "0m";
const std::string RED = "1m";
const std::string GREEN = "2m";
const std::string YELLOW = "3m";
const std::string BLUE = "4m";
const std::string WHITE = "7m";
const std::string GRAY = "8m";
const std::string PURPLE = "5m";
const std::string CYAN = "6m";
const std::string LIGHT_GRAY = "9m";
const std::string DARK_GRAY = "90m";
const std::string LIGHT_RED = "91m";
const std::string LIGHT_GREEN = "92m";
const std::string LIGHT_YELLOW = "93m";
const std::string LIGHT_BLUE = "94m";
const std::string LIGHT_PURPLE = "95m";
const std::string LIGHT_CYAN = "96m";
const std::string LIGHT_WHITE = "97m";
const std::string LIGHT_BLACK = "30m";
const std::string LIGHT_RED2 = "31m";
const std::string LIGHT_GREEN2 = "32m";
const std::string BOLD = "\033[1;3";
const std::string REGULAR = "\033[0;3";
const std::string UNDERLINE = "\033[4;3";
const std::string BACKGROUND = "\033[4";

/*!
 * \brief colouredString. Print coloured string
 * in terminal.
 * \param str. Input string.
 * \param colour. Colour option: BLACK, RED,
 * GREEN, YELLOW, BLUE, WHITE.
 * \param option. Char type option: BOLD, REGULAR,
 * UNDERLINE.
 * \return
 */
std::string colouredString(std::string str, std::string colour, std::string option);

#endif