string empty_string
string def_string "Hello world!"
string def_string_delimiter 'Hello"world!'
string def_string_delimiter2 "Hello'world!"
string<=22 ub_string
string<=22 ub_def_string "Upper bounded string."
string<=5[3] ub_string_static_array_value
string<=5[<=10] ub_string_ub_array_value
string<=5[] ub_string_dynamic_array_value
string[] def_string_dynamic_array_value ["What", "a", "wonderful", "world", "!"]
string[3] def_string_static_array_value ["Hello", "World", "!"]
string[<=10] def_string_bounded_array_value ["Hello", "World", "!"]
