use lazy_static::lazy_static;
use x86_64::structures::idt::InterruptDescriptorTable;
use x86_64::structures::idt::InterruptStackFrame;

fn outb(port: u16, val: u8) {
    unsafe {
        asm!("outb %al, %dx", in("al") val, in("dx") port, options(att_syntax));
    }
}

extern "x86-interrupt" fn breakpoint_handler(stack_frame: InterruptStackFrame) {
    unsafe {
        outb(0x80, 0x11);
    }
    panic!("Exception: Breakpoint.\r\n{:#?}", stack_frame);
}

extern "x86-interrupt" fn double_fault_handler(
    stack_frame: InterruptStackFrame,
    _error_code: u64,
) -> ! {
    panic!("Exception: Double fault.\r\n{:#?}", stack_frame);
}

extern "x86-interrupt" fn boom_handler(stack_frame: InterruptStackFrame, _error_code: u64) {
    panic!("BOOOOOM!\r\n{:#?}", stack_frame);
}

extern "x86-interrupt" fn stack_handler(stack_frame: InterruptStackFrame) {
    panic!("NOOOOOOO\r\n");
}

extern "x86-interrupt" fn interrupt_handler(stack_frame: InterruptStackFrame) {
    panic!("Interrupt.\r\n{:#?}", stack_frame);
}

//lazy_static! {
//    static ref IDT: InterruptDescriptorTable = {
//        let mut idt = InterruptDescriptorTable::new();
//        idt.breakpoint.set_handler_fn(breakpoint_handler);
//        idt.double_fault.set_handler_fn(double_fault_handler);
//        idt.divide_error.set_handler_fn(divide_error_handler);
//        idt
//    };
//}

pub fn init_idt() {
    //    IDT.load();
    unsafe {
        let mut idt = 0x100000 as *mut InterruptDescriptorTable;
        (*idt).breakpoint.set_handler_fn(breakpoint_handler);
        (*idt).double_fault.set_handler_fn(double_fault_handler);

        (*idt).segment_not_present.set_handler_fn(boom_handler);
        (*idt).stack_segment_fault.set_handler_fn(boom_handler);
        (*idt).general_protection_fault.set_handler_fn(boom_handler);

        (*idt).invalid_opcode.set_handler_fn(stack_handler);

        (*idt).x87_floating_point.set_handler_fn(stack_handler);
        // (*idt).simd_floating_point.set_handler_fn(stack_handler);
        // (*idt).divide_error.set_handler_fn(stack_handler);

        (*idt)[32].set_handler_fn(interrupt_handler);
        (*idt).load();
    }
}
