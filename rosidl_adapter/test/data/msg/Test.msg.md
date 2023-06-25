# Test Msg

Msg for testing parsing done by the `rosidl_adapter` package


It is typical for a comment to describe the entire message as a whole and then to have a comment right above  single field to describe that field as follows:

```
# msg level doc

# field level doc
bool bool_value
```

The test message definition has multiple fields declared in varying types. The following field is commented on the same line as the field instead of the line right before the field.

```
byte byte_value  # field level doc, style 2
```

Some fields are commented with two different styles
```
# combined styles
char char_value # combined styles, part 2
```

The remaining fields are not commented

```
float32 float32_value
float64 float64_value
int8 int8_value
uint8 uint8_value
int16 int16_value
uint16 uint16_value
int32 int32_value
uint32 uint32_value
int64 int64_value
uint64 uint64_value
```

See the [test_tangle_markdown_to_rosidl](../../test_tangle_markdown_to_rosidl.py) test case.
