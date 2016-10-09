

'''
Interrupts(Timer):
timer 1: 1s heart beat (check self status. See something? hear something?)
timer 2: ?

使用hardirq
http://blog.csdn.net/zhangskd/article/details/21992933

在include/linux/sched.h里声明。
 request_irq()调用的定义：
int request_irq(unsigned int irq,
 void (*handler)(int irq, void *dev_id, struct pt_regs *regs),
 unsigned long irqflags,
 const char * devname,
 oid *dev_id  );
 irq:         要申请的硬件中断号。在Intel平台，范围是0～15。
 handler: 向系统登记的中断处理函数。这是一个回调函数，中断发生时，系统掉用这个函数，传入的参数包
 括硬件中断号,device id,寄存器值。dev_id就是下面的request_irq时传递给系统的参数dev_id。
 irqflags: 中断处理的一些属性。比较重要的有SA_INTERRUPT,标明中断处理程序是快速处理程序（设置
 SA_INTERRUPT）还是慢速处理程序（不设置SA_INTERRUPT）。快速处理程序被调用时屏蔽
 所有中断。慢速处理程序不屏蔽。还有一个 SA_SHIRQ属性，设置了以后运行多个设备共享中
 断。
 dev_id:  中断共享时会用到。一般设置为这个设备的device结构本身或者NULL。中断处理程序可以用
 dev_id找到相应的控制这个中断的设备，或者用irq2dev_map找到中断对应的设备。
 
 
void free_irq(unsigned int irq, void *dev_id):
 不再使用已注册的中断服务时，使用 free_irq() 函数将其从内核注销掉。该函数在 2.4 内核和 2.6
 内核中原型相同:
#include <linux/interrupt.h>
                       void free_irq (unsigned int irq, void *dev_id);
 irq       :   是将要注销掉的中断服务函数的中断号；
 dev_id:  指定与request_irq() 函数中使用的 dev_id 值相同的值。

'''

'''
Soft Interrupts:
1. Action block
2. Sensor abnormal data
3.
'''

