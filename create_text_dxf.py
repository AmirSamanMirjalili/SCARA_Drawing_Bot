import ezdxf
import numpy as np

def create_letter_H(msp, start_x, start_y, height=10, width=5):
    # Vertical lines
    msp.add_line((start_x, start_y), (start_x, start_y + height))  # Left vertical
    msp.add_line((start_x + width, start_y), (start_x + width, start_y + height))  # Right vertical
    # Horizontal line
    msp.add_line((start_x, start_y + height/2), (start_x + width, start_y + height/2))  # Middle
    return width + 2  # Return width plus spacing

def create_letter_I(msp, start_x, start_y, height=10, width=2):
    # Vertical line
    msp.add_line((start_x + width/2, start_y), (start_x + width/2, start_y + height))
    return width + 2

def create_letter_E(msp, start_x, start_y, height=10, width=5):
    # Vertical line
    msp.add_line((start_x, start_y), (start_x, start_y + height))
    # Horizontal lines
    msp.add_line((start_x, start_y), (start_x + width, start_y))  # Bottom
    msp.add_line((start_x, start_y + height/2), (start_x + width, start_y + height/2))  # Middle
    msp.add_line((start_x, start_y + height), (start_x + width, start_y + height))  # Top
    return width + 2

def create_letter_L(msp, start_x, start_y, height=10, width=5):
    # Vertical line
    msp.add_line((start_x, start_y), (start_x, start_y + height))
    # Horizontal line
    msp.add_line((start_x, start_y), (start_x + width, start_y))
    return width + 2

def create_letter_O(msp, start_x, start_y, height=10, width=5):
    # Create a rectangle
    msp.add_line((start_x, start_y), (start_x + width, start_y))  # Bottom
    msp.add_line((start_x + width, start_y), (start_x + width, start_y + height))  # Right
    msp.add_line((start_x + width, start_y + height), (start_x, start_y + height))  # Top
    msp.add_line((start_x, start_y + height), (start_x, start_y))  # Left
    return width + 2

def create_text_dxf(text="HELLO", filename="text.dxf", height=10, letter_spacing=2):
    # Create a new DXF document
    doc = ezdxf.new('R2010')
    msp = doc.modelspace()

    # Starting position
    x, y = 0, 0
    
    # Letter creation functions dictionary
    letters = {
        'H': create_letter_H,
        'E': create_letter_E,
        'L': create_letter_L,
        'O': create_letter_O,
        'I': create_letter_I
    }
    
    # Create each letter
    for char in text.upper():
        if char in letters:
            # Create the letter and get its width
            width = letters[char](msp, x, y, height)
            # Move x position for next letter
            x += width + letter_spacing
        else:
            # For unsupported characters, add space
            x += 5

    # Save the DXF document
    doc.saveas(filename)
    print(f"Created text.dxf file with text: {text}")

if __name__ == "__main__":
    # Create a test file with "HELLO"
    create_text_dxf("HELLO", height=20)