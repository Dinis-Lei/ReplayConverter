import re
import sys

def printNode(node="", depth=0):
    # print("function -------------")
    # print(f"{node =}")
    line = ""
    
    # print(f"{node = }")    
    remainder = node
    count = 0
    idx_travelled = 0
        
    while True:
        
        if count < -1:
            break
        count += 1
        
        
        remainder = remainder.strip()
        
        # print("---------- while ---------")
        
        # print(f"{depth = }")
        
        parenthesis_close_position = re.search("[)]", remainder).start()
        try:
            parenthesis_start_position = re.search("[(]", remainder).start()
        except AttributeError:
            no_next_open = True
        
        # print(f"sussy {remainder = }")
        
        no_next_open = False
        
        try:
            next_parenthesis_start_position = re.search("[(]", remainder[1:]).start()
         
        except AttributeError:
            no_next_open = True
        
        
        if no_next_open or parenthesis_close_position < parenthesis_start_position:
            # print(f"end if, {remainder[:ma] = }")
            remainder = remainder[1:]
            break
            
        next_parenthesis_close_position = re.search("[)]", remainder[1:]).start()
        
        
        hasClosed = next_parenthesis_close_position < next_parenthesis_start_position
        idx = next_parenthesis_close_position+2
        
      
        mi = min(next_parenthesis_close_position, idx)
        ma = max(next_parenthesis_close_position, idx)
        
        # Two closes in a row
       
        
        line += depth*"\t"
        line += remainder[:parenthesis_start_position]
        
        # print(f"{remainder[:ma] = }")
        
        if hasClosed:
            # print("')' found")
            line += remainder[:idx] + "\n"
            # print(f"{line = }")
            
            idx_travelled += idx
            # print(f"sus {remainder[:idx] = }")
            remainder = remainder[idx:]
            
            
        else:
            # print('( found')
            line += remainder[:next_parenthesis_start_position] + "\n"
            # print(f"{line = }")
            idx_travelled += next_parenthesis_start_position
            
            text, next_remainder = printNode(remainder[next_parenthesis_start_position:], depth=depth+1)
            line += text + depth*"\t" + ")\n"
            remainder = next_remainder
                        
        
    return line, remainder


# # Line fecha na mesma linha
# if next_parenthesis_close_position < next_parenthesis_start_position or depth == 10:
#     retIdx = next_parenthesis_close_position+1
#     line += remainder[:retIdx]
#     return line, retIdx

# line += f"{remainder[:next_parenthesis_start_position]}\n"
# newline, nextIdx = printNode(output, remainder[next_parenthesis_start_position:], depth+1)
# line += newline
# line += printNode(output, remainder)
# return line
    
def main(args):
    
    if len(args) != 2:
        print("USAGE: python convertTestline.py <filename>")
        return
    
    filename = args[1]
    
    with open(filename, "r") as inp:
        with open(f"{filename}_converted", "w") as out:
            time = inp.readline()
            out.write(time)
            while True:
                line = inp.readline()
                out.write("\n ------ New Line ------ \n\n")
                if line:
                    line_converted = printNode(line)
                    out.write(line_converted[0])
                else: break
            print("Finished convertion!")
            

if __name__ == "__main__":
    main(sys.argv)