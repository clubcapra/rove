from typing import AnyStr, Any, List, Callable, Union, Type, Dict, Tuple

Validator = Callable[[Union[str, bool, int, float]], bool]
Parser = Callable[[str], Tuple[bool, Union[str, bool, int, float]]]

def cinput(prompt:AnyStr, type:Type, validator:Union[Validator, None] = None, options:AnyStr = None, parser:Parser = None) -> Union[str, int, float, bool, Any]:
    """Custom input
    prompt: Message to show as a prompt
    type: Class type to convert the input str
    validator: Callable returning a boolean indicating if the input is valid
    options: In certain cases could be used to display the description/options once while only repeating the prompt after a failed attempt
    returns: The converted return value form input
    """
    while True:
        if options:
            print(options)
            print()
        try:
            res = None
            if parser:
                b, p = parser(input(prompt))
                if not b:
                    print("Invalid value")
                    continue
                res = type(p)
            else:
                res = type(input(prompt))
        except ValueError:
            print(f"Invalid value type, expected {type}")
        except InterruptedError as e:
            print("Skipping")
            raise e
        if validator:
            v = validator(res)
            if v:
                return res
            else:
                print("Invalid value")
        else:
            if res is not None:
                return res

def intRangeValidator(min:int, max:int) -> Validator:
    """Validator for an int within a range
    min: Inclusive
    max: Exclusive"""
    def validator(value:int):
        return min <= value and value < max
    return validator

def ynValidator(value:str):
    value = value.lower()
    return value in 'yn'

def positiveNumberValidator(value:Union[float, int]):
    return value >= 0

def intChoice(prompt:str, options:List[str]) -> Tuple[int, Union[None, str]]:
    """Displays a list of the options with a number to select them
    prompt: Prompt to display before the list of options
    options: List of str to display
    returns: (res, option)
    res: 0 if 'Back' was selected. (1 to len(options)) if a value was selected. Note this is not the index, it is 'index + 1'
    option: None if 'Back' was selected. Name of the option 'option[res-1]'"""
    opt = f"{prompt}\n\t0  -> Back"
    for i, o in enumerate(options):
        idx = str(i+1).ljust(2)
        opt += f"\n\t{idx} -> {o}"
    validator = intRangeValidator(0, len(options)+1)
    res = cinput(f"Input number (0-{len(options)}): ", int, validator, opt)
    if res == 0:
        return res, None
    return res,  options[res - 1]

def engParser(value:str):
    if len(value) == 0:
        return False, None
    n = value[-1]
    try:
        res = float(value.rstrip('munpKMG'))
    except ValueError:
        print(f"Invalid input, notations include [m, u, n, p, K, M, G]")
        return False, None
    except InterruptedError as e:
        print("Skipping")
        raise e
    
    if n.isalpha():
        if n == 'm':
            res /= 1000
        if n == 'u':
            res /= 1000000
        if n == 'n':
            res /= 1000000000
        if n == 'p':
            res /= 1000000000000
        if n == 'K':
            res *= 1000
        if n == 'M':
            res *= 1000000
        if n == 'G':
            res *= 1000000000
    return True, res

def engNotationInput(prompt:str):
    return cinput(prompt, float, positiveNumberValidator, None, engParser)

Menu = Union[Callable, Dict[str, Union[Callable,Dict[str,Any]]]]

def menuInput(menu:Menu, name:str):
    stk = []
    while True:
        if callable(menu):
            print(name)
            menu()
            menu = stk.pop()
        if isinstance(menu, dict):
            r, opt = intChoice(f"{name}>", list(menu.keys()))
            if r == 0:
                if len(stk) == 0:
                    return
                menu = stk.pop()
                continue
            stk.append(menu)
            menu = menu[opt]
