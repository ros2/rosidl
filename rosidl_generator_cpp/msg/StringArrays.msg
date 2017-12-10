#string<=5[3] ub_string_static_array_value
#string<=5[<=10] ub_string_ub_array_value
#string<=5[] ub_string_dynamic_array_value
string[] string_dynamic_array_value
string[3] string_static_array_value
string[<=10] string_bounded_array_value
string[] def_string_dynamic_array_value ["What", "a", "wonderful", "world", "!"]
string[3] def_string_static_array_value ["Hello", "World", "!"]
string[<=10] def_string_bounded_array_value ['Hello', 'World', "!"]
# hmmm initialization fails on quotes for some reason
# string[] def_various_quotes ["H\"el'lo", 'Wo\'r"ld']
# though the following one workds o_O
# string[] def_various_quotes ["Hello", 'W"o\'rld']
# issue seem to be with double quoted strings:
#  single quotes or escaped double quotes
# string[] def_various_quotes ["He'llo", 'W"o\'rld']
string[] def_various_commas [,"Hel,lo", ',World', abcd,,   , "!,",]
