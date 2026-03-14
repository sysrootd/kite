#include <stdlib.h>

#include "stm32f4xx.h"
#include "sched.h"
#include "mem.h"
#include "uart.h"

uint32_t global_tick;                          // systick timer counter

TCB_t *current_running_node;                   // currently executing task node
TCB_t *head_node;                               // node pointer that holds first task(idle task) node addr
TCB_t *link_node;                               // node pointer that follow new node, lastly it will link with head node(make it circular list)

uint32_t *new_task_psp = STACK_START;           // stack pointer for new task
uint32_t *next_task_psp = STACK_START;          // next free stack location
uint32_t msp_start;                              //var that holds final tasks next_task_psp as msp

static inline void request_context_switch(void)
{
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk; //(1UL << 28) in bare metal,trigger PendSV exception,macro is from cmsis
}

void idle_task(void)
{
    while (1) {
        __asm volatile("WFI");                   // wait for interrupt
    }
}

void core_faults_init(void)
{
    // enable memory, bus, and usage fault handlers
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk; //(1 << 16); SCB_SHCSR_MEMFAULTENA_Msk these macros from cmsis
    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk; //(1 << 17);
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk; //(1 << 18);
}

__attribute__((naked)) void scheduler_init(void)
{
    msp_start = (uint32_t)next_task_psp;           // save stack top
    link_node->next_tcb_node = head_node;          // Make circular list

    __asm volatile(
        "PUSH {LR}                    \n"          // save return address
        "BL   core_faults_init        \n"          // enable fault handlers
        "POP  {LR}                    \n"          // restore return address
        "LDR  R0, =msp_start          \n"          // load &msp_start
        "LDR  R0, [R0]                \n"          // load msp_start value
        "MSR  MSP, R0                 \n"          // set MSP to stack top
        "ISB                          \n"          // flush pipeline
        "PUSH {LR}                    \n"          // save return address
        "BL   task_stack_init         \n"          // init all task stacks
        "POP  {LR}                    \n"          // restore return address
        "BX   LR                      \n"          // return
    );
}

__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile(
        "BL __get_psp        \n"                    // get current task PSP
        "MSR PSP, R0         \n"                    // set PSP to task stack
        "LDMIA R0!, {R4-R11} \n"                    // Restore R4-R11 from stack
        "MSR PSP, R0         \n"                    // update PSP after restore
        "LDR LR, =0xFFFFFFFD \n"                    // set EXC_RETURN for thread mode+PSP
        "BX LR               \n"                    // return to task
    );
}

__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        "MRS R0, PSP           \n"                  // get current PSP
        "STMDB R0!, {R4-R11}   \n"                  // save R4-R11 on stack
        "PUSH {LR}             \n"                  // save EXC_RETURN
        "BL __set_psp          \n"                  // save updated PSP to TCB
        "BL cooperative_sched  \n"                  // pick next task to run
        "BL __get_psp          \n"                  // get next task's PSP
        "LDMIA R0!, {R4-R11}   \n"                  // restore next task's R4-R11
        "MSR PSP, R0           \n"                  // set PSP to next task stack
        "POP {LR}              \n"                  // restore EXC_RETURN
        "BX LR                 \n"                  // return to next task
    );
}

void scheduler_start(void)
{
    systick_init();                                  // Start system tick timer
    __asm volatile ("svc 0");                        // Trigger SVC to start first task
}

void systick_init(void)
{
    SysTick_Config(SYSTEM_CLK / TICK_HZ);               // configure SysTick timer(cmsis)

    NVIC_SetPriority(PendSV_IRQn, 0xFF);             // lowest priority for PendSV(cmsis)
    NVIC_SetPriority(SysTick_IRQn, 0x00);            // highest priority for SysTick(cmsis)
}

void SysTick_Handler(void)
{
    global_tick++;                                    // increment system tick
    task_wake();                                      // wake any sleeping tasks
    request_context_switch();                         // request context switch
}

void schedule(void)
{
    request_context_switch();                         // request context switch
}

void task_stack_init(void)
{
    TCB_t *iter = head_node;
    
    if (head_node == NULL) {
        return;
    }
    
    do {
        //ensure 8-byte alignment for stack pointer
        uint32_t *stack_top = iter->psp_value;
        uint32_t aligned_psp = ((uint32_t)stack_top) & ~0x7;
        
        //initialize stack frame with initial register values
        uint32_t *stack = (uint32_t *)aligned_psp;
        
        *(--stack) = 0x01000000;                       // xPSR with Thumb bit set
        *(--stack) = (uint32_t)iter->task_handler;     // Program counter
        *(--stack) = 0xFFFFFFFD;                       // lr with EXC_RETURN value
        *(--stack) = 0x0000000C;                       // r12
        *(--stack) = 0x00000003;                       // r3
        *(--stack) = 0x00000002;                       // r2
        *(--stack) = 0x00000001;                       // r1
        *(--stack) = 0x00000000;                       // r0
        
        // save r4-r11 (initialized to zero)
        for (int i = 0; i < 8; i++) {
            *(--stack) = 0;
        }
        
        iter->psp_value = stack;                       // update task's PSP
        iter = iter->next_tcb_node;

    } while (iter != head_node);
}

uint32_t *find_stack_area(uint32_t stack_size_in_words)
{
    //ensure even number of words for alignment
    stack_size_in_words = (stack_size_in_words + 1) & ~1;

    new_task_psp = next_task_psp;                      //current stack top for new task
    next_task_psp -= stack_size_in_words;              // move next pointer for next task

    return new_task_psp;
}

TCB_t *alloc_new_tcb_node(void)
{
    TCB_t *new_tcb_node = TCB_pool(sizeof(TCB_t)); //allocate mem for new tcb node and get that addr

    if (new_tcb_node == NULL) {
        return NULL;
    }

    new_tcb_node->next_tcb_node = NULL;
    return new_tcb_node;
}

void idle_task_init(void)
{
    TCB_t *idle_tcb_node = alloc_new_tcb_node();

    if (idle_tcb_node == NULL) {
        return;
    }

    uint32_t *task_stack_start = find_stack_area(IDLE_TASK_STACK_SIZE);

    idle_tcb_node->block_count = 0;
    idle_tcb_node->current_state = TASK_WAKE;          // idle task always ready
    idle_tcb_node->psp_value = task_stack_start;
    idle_tcb_node->task_handler = idle_task;
    idle_tcb_node->waiting_on = NULL;

    current_running_node = idle_tcb_node;              // start with idle task
    link_node = idle_tcb_node;                          // first node in list
    head_node = idle_tcb_node;                          // head of circular list
}

void task_init(uint8_t task_priority, void (*task_handle)(void), uint32_t stack_size_in_words)
{
    //initialize idle task first if no tasks exist
    if(new_task_psp == next_task_psp) {
        idle_task_init();
    }

    TCB_t *tcb_node = alloc_new_tcb_node();

    if (tcb_node == NULL) {
        return;
    }

    uint32_t *task_stack_start = find_stack_area(stack_size_in_words);

    tcb_node->block_count = 0;
    tcb_node->current_state = TASK_WAKE;               // Task starts ready to run
    tcb_node->psp_value = task_stack_start;
    tcb_node->task_handler = task_handle;
    tcb_node->waiting_on = NULL;

    //add task to circular list
    link_node->next_tcb_node = tcb_node;
    link_node = tcb_node;
}

uint32_t __get_psp(void)
{
    return (uint32_t)current_running_node->psp_value;  // Get current task's PSP
}

void __set_psp(uint32_t current_psp_value)
{
    current_running_node->psp_value = (uint32_t *)current_psp_value;  // Save PSP to TCB
}

void cooperative_sched(void)
{
    TCB_t *candidate = current_running_node->next_tcb_node;

    //search for next ready task
    do
    {
        if(candidate->current_state == TASK_WAKE)
        {
            current_running_node = candidate;          //switch to ready task
            return;
        }

        candidate = candidate->next_tcb_node;

    } while(candidate != current_running_node);

    //if no ready task found, run head task (idle task)
    current_running_node = head_node;
}

void task_delay(uint32_t tick_count)
{
    ENTER_CRITICAL();                                   //disable interrupts

    if (current_running_node != NULL) {
        current_running_node->block_count = global_tick + tick_count;  // Wakeup time
        current_running_node->current_state = TASK_SLEEP;              // Put to sleep
        schedule();                                                      // Request switch
    }

    EXIT_CRITICAL();                                    //enable interrupts
}

void task_wake(void)
{
    TCB_t *iter = head_node;

    //check all tasks for expired sleep timers
    do {
        if (iter->current_state == TASK_SLEEP) {
            if (iter->block_count <= global_tick) {    // sleep time expired?
                iter->current_state = TASK_WAKE;       // wake up task
            }
        }
        iter = iter->next_tcb_node;
    } while (iter != head_node && iter != NULL);
}

void semaphore_init(semaphore_t *sem, int32_t initial_count)
{
    sem->count = initial_count;                         // initialize semaphore count
}

void semaphore_wait(semaphore_t *sem)
{
    ENTER_CRITICAL();

    sem->count--;                                       //decrement semaphore

    if (sem->count < 0)                                 // resource not available?
    {
        current_running_node->waiting_on = sem;         // block on this semaphore
        current_running_node->current_state = TASK_BLOCKED;
        schedule();                                      // switch to another task
    }

    EXIT_CRITICAL();
}

void semaphore_post(semaphore_t *sem)
{
    ENTER_CRITICAL();

    sem->count++;                                       //increment semaphore

    if (sem->count <= 0)                                //is tasks waiting or not?
    {
        TCB_t *iter = head_node;

        // find a task waiting on this semaphore
        do
        {
            if (iter->current_state == TASK_BLOCKED &&
                iter->waiting_on == sem)
            {
                iter->waiting_on = NULL;
                iter->current_state = TASK_WAKE;       // unblock task
                request_context_switch();               // request context switch
                break;
            }

            iter = iter->next_tcb_node;

        } while (iter != head_node);
    }

    EXIT_CRITICAL();
}

void mutex_init(mutex_t *m)
{
    m->locked = 0;                                      //mutex initially unlocked
    m->owner  = NULL;                                   //no owner
}

void mutex_lock(mutex_t *m)
{
    ENTER_CRITICAL();

    if (m->locked == 0)                                 // is Mutex available?
    {
        m->locked = 1;                                  //lock it
        m->owner  = current_running_node;               // set owner
    }
    else                                                // Mutex already locked
    {
        current_running_node->waiting_on = m;           // block on this mutex
        current_running_node->current_state = TASK_BLOCKED;
        schedule();                                      //switch to another task
    }

    EXIT_CRITICAL();
}

void mutex_unlock(mutex_t *m)
{
    ENTER_CRITICAL();

    if (m->owner == current_running_node)               //nly owner can unlock
    {
        TCB_t *iter = head_node;
        TCB_t *next_owner = NULL;

        //find next waiting task for this mutex
        do
        {
            if (iter->current_state == TASK_BLOCKED &&
                iter->waiting_on == m)
            {
                next_owner = iter;
                break;
            }

            iter = iter->next_tcb_node;
        } while (iter != head_node);

        if (next_owner)                                 //handover to next task
        {
            next_owner->waiting_on = NULL;
            next_owner->current_state = TASK_WAKE;     // Unblock next task
            m->owner = next_owner;                      // transfer ownership
            request_context_switch();                    // request context switch
        }
        else                                            // no tasks waiting
        {
            m->locked = 0;                               // unlock mutex
            m->owner  = NULL;
        }
    }

    EXIT_CRITICAL();
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "TST lr, #4       \n"                           // check which stack was used
        "ITE EQ           \n"                           // if-Then-Else condition
        "MRSEQ r0, MSP    \n"                           // use MSP if equal
        "MRSNE r0, PSP    \n"                           // use PSP if not equal
        "B hardfault      \n"                           // branch to C handler
    );
}

void hardfault(uint32_t *stack)
{
    // extract registers values from stack frame
    uint32_t r0  = stack[0];
    uint32_t r1  = stack[1];
    uint32_t r2  = stack[2];
    uint32_t r3  = stack[3];
    uint32_t r12 = stack[4];
    uint32_t lr  = stack[5];
    uint32_t pc  = stack[6];
    uint32_t psr = stack[7];

    // Print fault info
    uart_printf(USART2,"HardFault\r\n");
    uart_printf(USART2,"R0  %x\r\n",r0);
    uart_printf(USART2,"R1  %x\r\n",r1);
    uart_printf(USART2,"R2  %x\r\n",r2);
    uart_printf(USART2,"R3  %x\r\n",r3);
    uart_printf(USART2,"R12 %x\r\n",r12);
    uart_printf(USART2,"LR  %x\r\n",lr);
    uart_printf(USART2,"PC  %x\r\n",pc);
    uart_printf(USART2,"PSR %x\r\n",psr);

    while(1);
}

void MemManage_Handler(void)
{
    uart_printf(USART2,"MemManage Fault\r\n");
    uart_printf(USART2,"MMFSR %x\r\n",SCB->CFSR & 0xFF);        //MemManage status
    uart_printf(USART2,"MMFAR %x\r\n",SCB->MMFAR);              //fault address

    while(1);
}

void BusFault_Handler(void)
{
    uart_printf(USART2,"Bus Fault\r\n");
    uart_printf(USART2,"BFSR %x\r\n",(SCB->CFSR >> 8) & 0xFF);  // Bus fault status
    uart_printf(USART2,"BFAR %x\r\n",SCB->BFAR);                 //fault address

    while(1);
}

void UsageFault_Handler(void)
{
    uart_printf(USART2,"Usage Fault\r\n");
    uart_printf(USART2,"UFSR %x\r\n",(SCB->CFSR >> 16) & 0xFFFF); // Usage fault status

    while(1);
}