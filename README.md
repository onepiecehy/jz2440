# u-boot启动流程 

*version: uboot-2016-11*

## u-boot.bin生成路径

搜索Makefile文件，当我们执行make操作时，相应的依赖项如下所示：

```makefile
ALL-y += u-boot.srec u-boot.bin u-boot.sym System.map u-boot.cfg binary_size_check

	-| u-boot.bin: u-boot-nodtb.bin FORCE
		-| u-boot-nodtb.bin: u-boot FORCE 	$(call if_changed,objcopy)	$(call DO_STATIC_RELA,u-boot,u-boot-nodtb.bin,$(CONFIG_SYS_TEXT_BASE))
		-| u-boot:	$(u-boot-init) $(u-boot-main) u-boot.lds FORCE
			-| u-boot-init := $(head-y)
			-| u-boot-main := $(libs-y)
				-| arch/arm/Makefile:74:  head-y := arch/arm/cpu/$(CPU)/start.o
				-| ./Makefile:638 libs-y += lib/
			-| u-boot.lds: $(LDSCRIPT) prepare FORCE
```

我们最终需要的是u-boot.bin，而u-boot.bin又由`u-boot-init`,`u-boot-main`,以及连接文件`u-boot.lds`组成。
1.u-boot-init *：包含了start.o，位于生成文件的头部位置，用于对系统进行初始化处理，另外有一些因为内存限制的原因需要预先处理的代码，也需要在start.o中实现。
2.* u-boot-main *：包含了其他编译生成的文件，$(libs-y)在Makefile中有定义，根据宏定义会有些取舍。在libs-y的组成中我们会发现有一些宏需要确定，例如：`VENDOR`,`BOARDDIR`等。搜索变量，可以发现在顶层目录下的config.mk有类似定义的地方。

```makefile
./config.mk：
ARCH := $(CONFIG_SYS_ARCH:"%"=%)
CPU := $(CONFIG_SYS_CPU:"%"=%)
BOARD := $(CONFIG_SYS_BOARD:"%"=%)
VENDOR := $(CONFIG_SYS_VENDOR:"%"=%)
SOC := $(CONFIG_SYS_SOC:"%"=%)
CPUDIR=arch/$(ARCH)/cpu$(if $(CPU),/$(CPU),)
BOARDDIR = $(VENDOR)/$(BOARD)

sinclude $(srctree)/arch/$(ARCH)/config.mk	# include architecture dependend rules
sinclude $(srctree)/$(CPUDIR)/config.mk		# include  CPU	specific rules
sinclude $(srctree)/$(CPUDIR)/$(SOC)/config.mk	# include  SoC	specific rules
sinclude $(srctree)/board/$(BOARDDIR)/config.mk	# include board specific rules

export ARCH CPU BOARD VENDOR SOC CPUDIR BOARDDIR
```

继续搜索`CONFIG_SYS_`开头的宏，可以发现在`board/samsung/jz2440（目标板）/Kconfig` 中找到定义。

```makefile
if TARGET_JZ2440//对应着CONFIG_TARGET_JZ2440，在jz2440_defconfig中定义

config SYS_BOARD
	default "jz2440"

config SYS_VENDOR
	default "samsung"

config SYS_SOC
	default "s3c24x0"

config SYS_CONFIG_NAME
	default "jz2440"

endif
```

可以确定的是当我们执行`make jz2440_defconfig`时，执行了某些的操作使`board/samsung/jz2440（目标板）/Kconfig`生效了，我们来分析一下，首先在Makefile能找到的唯一与jz2440_defconfig匹配的只有`%config`:

```makefile
%config: scripts_basic outputmakefile FORCE
	$(Q)$(MAKE) $(build)=scripts/kconfig $@

#而在scripts/Kbuild.include中有这样的定义：
# $(Q)$(MAKE) $(build)=dir
build := -f $(srctree)/scripts/Makefile.build obj

#因此可以展开为：
jz2440_defconfig：
	$(Q)$(MAKE) -f $(srctree)/scripts/Makefile.build obj=scripts/kconfig jz2440_defconfig
```

*scripts/Makefile.build*：

```makefile
# Modified for U-Boot
-include include/config/auto.conf //在这里发现了我们想要的宏定义

#在auto.conf.cmd中有如下定义,$(deps_config)包含了源码目录下的所有Kconfig文件:
include/config/auto.conf: \
	$(deps_config)

```

至此，可以判断，因为所有的Kconfig都被包含了，所以在`make jz2440_defconfig`时定义的CONFIG_TARGET_JZ2440，使得`board/samsung/jz2440（目标板）/Kconfig`中的配置生效了。

3. - u-boot.lds * :按着以下的路径寻找，找到的第一个即作为实际的连接文件。

```makefile
ifndef LDSCRIPT//依次往下找
	ifeq ($(wildcard $(LDSCRIPT)),)
		LDSCRIPT := $(srctree)/board/$(BOARDDIR)/u-boot.lds
	endif
	ifeq ($(wildcard $(LDSCRIPT)),)
		LDSCRIPT := $(srctree)/$(CPUDIR)/u-boot.lds
	endif
	ifeq ($(wildcard $(LDSCRIPT)),)
		LDSCRIPT := $(srctree)/arch/$(ARCH)/cpu/u-boot.lds
	endif
endif
```

arch/arm/cpu/u-boot.lds：

```makefile
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
SECTIONS
{	
/* 指定可执行image文件的全局入口点，通常这个地址都放在ROM（flash）0x0的位置。必须使编译器知道这个地址，通常都是修改此处来完成 */
	. = 0x00000000;//从0x0位置开始

	. = ALIGN(4);//代码以4字节对齐
	.text :
	{
		*(.__image_copy_start)
		/* u-boot将自己copy到RAM，此为需要拷贝的程序的start */
		*(.vectors)
		CPUDIR/start.o (.text*)
		*(.text*)
	}
	. = ALIGN(4);

	.rodata : { *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*))) }/*只读数据段*/
	. = ALIGN(4);

	.data : {/*代码段*/
		*(.data*)
	}
	. = ALIGN(4);

	. = .;

	. = ALIGN(4);
	.u_boot_list : {
		KEEP(*(SORT(.u_boot_list*)));
		/* .data 段结束后，紧接着存放u-boot自有的一些function，例如u-boot command等*/
	}

	. = ALIGN(4);

	.__efi_runtime_start : {
		*(.__efi_runtime_start)

	}

	.efi_runtime : {
		*(efi_runtime_text)
		*(efi_runtime_data)
	}

	.__efi_runtime_stop : {
		*(.__efi_runtime_stop)
	}

	.efi_runtime_rel_start :
	{
		*(.__efi_runtime_rel_start)
	}

	.efi_runtime_rel : {
		*(.relefi_runtime_text)
		*(.relefi_runtime_data)
	}

	.efi_runtime_rel_stop :
	{
		*(.__efi_runtime_rel_stop)
	
	. = ALIGN(4);

	.image_copy_end :
	{
		*(.__image_copy_end)
	}

	.rel_dyn_start :
	{
		*(.__rel_dyn_start)
	}

	.rel.dyn : {
		*(.rel*)
	}
	/* 动态链接符存放在的段，只要给这里面对额符号加上一定的偏移，拷贝到内存中代码*/
	.rel_dyn_end :
	{
		*(.__rel_dyn_end)
	}
	/* 动态链接符段结束*/
	.end :
	{
		*(.__end)
	}

	_image_binary_end = .;
	/* bin文件结束 */
	
	/*
	 * Deprecated: this MMU section is used by pxa at present but
	 * should not be used by new boards/CPUs.
	 */
	. = ALIGN(4096);
	.mmutable : {
		*(.mmutable)
	}

/*
 * Compiler-generated __bss_start and __bss_end, see arch/arm/lib/bss.c
 * __bss_base and __bss_limit are for linker only (overlay ordering)
 */
	/* bss段的描述 */
	.bss_start __rel_dyn_start (OVERLAY) : {
		KEEP(*(.__bss_start));
		__bss_base = .;
	}

	.bss __bss_base (OVERLAY) : {
		*(.bss*)
		 . = ALIGN(4);
		 __bss_limit = .;
	}

	.bss_end __bss_limit (OVERLAY) : {
		KEEP(*(.__bss_end));
	}
	/* bss段的描述结束 */
	.dynsym _image_binary_end : { *(.dynsym) }
	.dynbss : { *(.dynbss) }
	.dynstr : { *(.dynstr*) }
	.dynamic : { *(.dynamic*) }
	.plt : { *(.plt*) }
	.interp : { *(.interp*) }
	.gnu.hash : { *(.gnu.hash) }
	.gnu : { *(.gnu*) }
	.ARM.exidx : { *(.ARM.exidx*) }
	.gnu.linkonce.armexidx : { *(.gnu.linkonce.armexidx.*) }
}
```

## u-boot.bin启动流程

从start.S开始分析,以下是start.S的主要流程：

```
reset:
	|-- 进入保护模式
	|-- 关闭看门狗
	|-- 关中断
	|-- #ifndef CONFIG_SKIP_LOWLEVEL_INIT
		|-- lowlevel_init[lowlevel_init.S]: dram设置
	|-- _main[crt0.S]
```

*ps*：当我们在内存中直接执行uboot时，由于此时dram肯定是可以用的，已经被当前运行的uboot初始化好了，不用也不可再进行初始化，否侧刚加载到内存中的uboot将会被擦除。此时需要定义宏`CONFIG_SKIP_LOWLEVEL_INIT`跳过dram初始化。

```assembly
main[arch/arm/lib/crt0.S]:
	|--- board_init_f_alloc_reserve[common/init]  
	|--- board_init_f_init_reserve[common/init]
	|--- board_init_f
	|--- relocate_code & relocate_vectors & clear bss
	|--- board_init_r
```

1. board_init_f_alloc_reserve
   GD是16字节的对齐的，堆栈分配从上往下分配，因为GD是最后分配的，所以返回的地址即为GD的首地址(GD内部地址是从下往上的，所以首地址为最低位地址)。

```c
 ulong board_init_f_alloc_reserve(ulong top)
{
	/* Reserve early malloc arena */
#if defined(CONFIG_SYS_MALLOC_F)
	top -= CONFIG_SYS_MALLOC_F_LEN;//如果定义了CONFIG_SYS_MALLOC_F_LEN，则要相应的减去占用的长度
#endif
	/* LAST : reserve GD (rounded up to a multiple of 16 bytes) */
	top = rounddown(top-sizeof(struct global_data), 16);//四舍五入为16 的倍数

	return top;
}

|--CONFIG_SYS_INIT_SP_ADDR
|--CONFIG_SYS_MALLOC_F
|--top(GD)
|--
```

2. board_init_f_init_reserve 
   有些架构的GD需要调用一次`arch_setup_gd()`，因此，在调用`arch_setup_gd()`之前，统一使用`gd_ptr`,从`arch_setup_gd()`返回之后才可以使用`gd->`。

```c
void board_init_f_init_reserve(ulong base)
{
	struct global_data *gd_ptr;
#ifndef _USE_MEMCPY
	int *ptr;
#endif

	/*
	 * clear GD entirely and set it up.
	 * Use gd_ptr, as gd may not be properly set yet.
	 */

	gd_ptr = (struct global_data *)base;
	/* zero the area */
#ifdef _USE_MEMCPY
	memset(gd_ptr, '\0', sizeof(*gd));
#else
	for (ptr = (int *)gd_ptr; ptr < (int *)(gd_ptr + 1); )
		*ptr++ = 0;
#endif
	/* set GD unless architecture did it already */
#if !defined(CONFIG_ARM) //arm架构不执行arch_setup_gd(),X86需要执行自己实现的arch_setup_gd()。
	arch_setup_gd(gd_ptr);
#endif
	/* next alloc will be higher by one GD plus 16-byte alignment */
	base += roundup(sizeof(struct global_data), 16);

	/*
	 * record early malloc arena start.
	 * Use gd as it is now properly set for all architectures.
	 */

#if defined(CONFIG_SYS_MALLOC_F)
	/* go down one 'early malloc arena' */
	gd->malloc_base = base;
	/* next alloc will be higher by one 'early malloc arena' size */
	base += CONFIG_SYS_MALLOC_F_LEN; //照代码的逻辑，此时base = CONFIG_SYS_INIT_SP_ADDR；
#endif
}
```

3. board_init_f：主要是实现了一些初始化工作，以及重定位的位置分配，其中在console_init_f之后才可以调用串口打印。代码中也是第一时间输出了cpu信息。

```c
void board_init_f(ulong boot_flags)
{
...
	if (initcall_run_list(init_sequence_f))
		hang();
...
}

static init_fnc_t init_sequence_f[] = {
...
	setup_mon_len,
	board_early_init_f,
	timer_init,		/* initialize timer */
	env_init,		/* initialize environment */
	init_baud_rate,		/* initialze baudrate settings */
	serial_init,		/* serial communications setup */
	console_init_f,		/* stage 1 init of console */
	print_cpuinfo,		/* display cpu info (and speed) */
	dram_init,		/* configure available RAM banks */
	setup_dest_addr,
	setup_machine,
	reserve_global_data,
	reserve_arch,
	reserve_stacks,
	setup_dram_config,
	show_dram_config,
	display_new_sp,
	setup_reloc,
	NULL,
};
```

4. - relocate_code & relocate_vectors & clear bss*：这一部分主要是根据board_init_f中定义的位置，将当前的uboot代码及变量搬到指定地点，并清除bss段。
     *ps*：笔者在实际移植过程中遇到了一个问题，每次代码执行到这一部分，就卡死了。后来通过修改内存大小`PHYS_SDRAM_1_SIZE`实现了uboot的加载。猜测是因为在重定位过程中，与运行中的uboot在内存区域产生了冲突，导致uboot没能顺利完成重定位。
5. - board_init_r*：与board_init_f一样，在这个阶段有一系列的函数需要依次执行，在移植过程中，遇到具体问题可以具体去看。下面是比较重要的几个函数，在需要支持NOR,NAND,NET时需要去做适配。最后uboot进入main loop，接收用户命令并执行。

```c
init_fnc_t init_sequence_r[] = {
	board_init,	/* Setup chipselects */
	initr_flash, //NOR flash
	initr_nand,  //nand flash
	initr_net,   //net
	run_main_loop,//main loop
}
```



## u-boot-2016.11 移植

------

- 编译器：arm-2014.05-29-arm-none-linux-gnueabi-i686-pc-linux-gnu.tar.bz2

------

一、准备环境

由于是移植到自己的开发板上，免不了需要做一些改动才能运行，我们将基于smdk2410进行修改工作，为了验证交叉编译环境是否可用，先编译一把smdk2410的u-boot，如果没有问题，说明编译环境是可以工作了。

二、拷贝smdk2410开发板相关文件，准备进行修改。

文件：

board/samsung/smdk2410 -->  board/samsung/jz2440

include/configs/smdk2410.h --> include/configs/jz22440.h

drivers/mtd/nand/s3c2410_nand.c --> drivers/mtd/nand/s3c2440_nand.c

configs/smdk2410_defconfig --> configs/jz2440_defconfig

内容：

1.上述新复制的文件中所有smdk2410，s3c2410 都改为 jz2440 ， s3c2440

2.arch/arm/Kconfig 中所有smdk2410，s3c2410 都改为 jz2440 ， s3c2440

3../scripts/config_whitelist.txt 中所有有smdk2410，s3c2410 的都添加相应的 jz2440 ， s3c2440，**需要注意的是这个文件会对文件内容排序，所以要注意顺序，放到该放的位置**。

==*阶段成果：使用jz2440_defconfig可以编译成功。*==



三、根据芯片手册，修改内容

------

arch/arm/cpu/arm920t/start.S

​	|-- lowlevel_init(board/samsung/jz2440/lowlevel_init.S)

​	|-- _main(arch/arm/lib/crt0.S)

_main执行顺序:

1.为调用board_init_f()设置初始环境。这个环境只提供一个堆栈和一个存储GD(“全局数据”)结构的位置，两者都位于一些可用的RAM (SRAM，锁定缓存……)中。在此上下文中，变量全局数据(无论是否初始化)都不可用;只有常量初始化的数据可用。在 board_init_f()被调用之前GD应该初始化为零

2.调用board_init_f ()。该函数为从系统RAM (DRAM、DDR…)执行准备硬件。由于系统RAM可能还不可用，因此board_init_f()必须使用当前GD来存储必须传递到后续阶段的任何数据。这些数据包括重新定位目标、未来堆栈和未来GD位置。

3.设置中间环境，其中堆栈和GD是由系统RAM中的board_init_f()分配的，但是BSS和初始化的非const数据仍然不可用。

4a.对于适当的U-Boot (不是SPL), 调用relocate_code(). 此函数将U-Boot从当前位置重新定位到由board_init_f()计算的重新定位目的地。

4b.对于SPL, board_init_f()只返回(到crt0)。SPL中没有代码重定位。

5.设置调用board_init_r()的最终环境。这个环境有BSS(初始化为0)、非const数据(初始化为它们的预期值)和系统RAM中的堆栈(SPL将堆栈和GD移动到RAM中是可选的，请参见CONFIG_SPL_STACK_R)。GD保留了board_init_f()设置的值。

6.对于适当的U-Boot (不是SPL)，一些cpu此时还有一些内存方面的工作要做，因此调用c_runtime_cpu_setup。

7.切换到 board_init_r().



------

调试记录：

1.修改完寄存器相关的值后，加载uboot，没有得到输出

解决：在jz2440.h中添加宏定义` #define CONFIG_SKIP_LOWLEVEL_INIT `,由于不添加宏，则会调用lowlevel_init将内存初始化，那么被加载到内存的u-boot.bin也会被清除，所以需要跳过。

2.系统挂死，添加DEBUG宏后，最后的打印显示board_init_f结束，但是board_init_r还未进入。

3.修改内存总大小之后，uboot启动成功，顺利进入main loop。

------

四、dm9000 配置

1.打开宏配置

```c
#define CONFIG_DRIVER_DM9000                    /*DM9000*/
#define CONFIG_DM9000_BASE                   0x20000000
#define DM9000_IO                            CONFIG_DM9000_BASE          
#define DM9000_DATA                         (CONFIG_DM9000_BASE + 4)
```

2.修改board_eth_init

```c
int board_eth_init(bd_t *bis)
{
	int rc = 0;
#ifdef CONFIG_CS8900
	rc = cs8900_initialize(0, CONFIG_CS8900_BASE);
#endif

#ifdef CONFIG_DRIVER_DM9000
	rc = dm9000_initialize(bis);
#endif
	return rc;
}
```



五、NOR flash配置

笔者开发板使用的是MX29LV160DB，与兼容AMD。

首先，根据数据手册可知其内部被分为了35个SECT，所以需要在头文件中修改宏定义的值：

```c
#define CONFIG_SYS_MAX_FLASH_SECT	(35)
```

其次，因为是与AMD兼容的，所以架构中AMD的代码大部分都可以直接使用，需要修改的地方是读到device的id后的匹配部分：

这部分官方代码中没有定义，需要我们手动添加一下：

```c
//drivers/mtd/jedec_flash.c
...
#define MX29LV160DB	0x2249	
...
static const struct amd_flash_info jedec_table[] = {
        #在表中添加如下字段
        #ifdef CONFIG_SYS_FLASH_LEGACY_512Kx16
        {
            .mfr_id		= (u16)MX_MANUFACT,
            .dev_id		= MX29LV160DB,
            .name		= "MXIC MX29LV160DB",
            .uaddr		= {
                [1] = MTD_UADDR_0x0555_0x02AA /* x16 */
            },
            .DevSize	= SIZE_2MiB,
            .CmdSet		= CFI_CMDSET_AMD_LEGACY,
            .NumEraseRegions= 4,
            .regions	= {
                ERASEINFO(0x10000, 31),
                ERASEINFO(0x08000, 1),
                ERASEINFO(0x02000, 2),
                ERASEINFO(0x04000, 1),
            }
        },
        #endif
	}
```



ps：笔者的JZ2440开发板需要将拨码开关拨到nor flash上才能使用nor flash，所以如果是nand flash启动方式的话，nor flash的id都读不到。

六、NAND FLASH配置

1.注释掉CONFIG_SYS_S3C2440_NAND_HWECC宏，使用软件ECC。

2.从s3c2410_nand.c拷贝过来的s3c2440_nand.c，需要将相关寄存器修改一下,下面是修改的宏定义，文件中其他设计到这些宏的地方都需要改。

```c
#define S3C2440_NFCONT_EN          (1<<0)
#define S3C2440_NFCONT_nFCE        (1<<1)
#define S3C2440_NFCONT_INITECC     (1<<4)

#define S3C2440_NFCONF_TACLS(x)    ((x)<<12)
#define S3C2440_NFCONF_TWRPH0(x)   ((x)<<8)
#define S3C2440_NFCONF_TWRPH1(x)   ((x)<<4)

#define S3C2440_ADDR_NALE 0x8
#define S3C2440_ADDR_NCLE 0xc
```



七、TF卡配置

庆幸的是大部分需要移植的工作在driver/mmc/s3c_sdi.c中已经做好了

1.在jz2440.h中调价以下宏定义

```c
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_S3C_SDI
```

2.添加检测卡的函数实现

```c
static int s3cmmc_priv_getcd(struct mmc *mmc)
{
    ulong mmc_cd;
    struct s3c24x0_gpio * const gpio = s3c24x0_get_base_gpio();
    
    mmc_cd = readl(&gpio->gpgcon) & ~(0x3<<16);
    writel(mmc_cd,&gpio->gpgcon);
    
    mmc_cd = (readl(&gpio->gpgdat)>>8) & 0x1;
    if(mmc_cd)
    	return 0;
    else
    	return 1;
}


int s3cmmc_initialize(...)
{
    ...
    if(!priv)
    	return -ENOMEM;
+	priv->getcd = s3cmmc_priv_getcd;
    ...
}
```

3.执行fatls mmc 0 可以查看tf卡中的文件

4.执行fatload mmc 0 32000000 <filename> 可以将文件加载到内存0x32000000的地方




# 移植 linux-4.20 到 jz2440 开发板

------

- 编译器：arm-2014.05-29-arm-none-linux-gnueabi-i686-pc-linux-gnu.tar.bz2
- 内核版本：linux-4.20.9

------

一、准备环境

内核中有线程的s3c2410_defconfig配置文件，我们将在其基础上进行移植。为了验证交叉编译环境是否可用，先用s3c2410_defconfig进行一次编译，如果没有问题，说明编译环境是可以工作了。

笔者在成功编译之前，根据提示先安装了几个文件：



二、拷贝smdk2410开发板相关文件，准备进行修改。

文件：

kernel/linux-4.20.9/arch/arm/mach-s3c24xx/mach-smdk2440.c -->  kernel/linux-4.20.9/arch/arm/mach-s3c24xx/mach-jz2440.c

内容：

1.上述新复制的文件中所有smdk2440都改为 jz2440 

2.boot/u-boot-2016.11/arch/arm/include/asm/mach-types.h 中添加 MACH_TYPE_JZ2440 = 1995。

3.kernel/linux-4.20.9/arch/arm/tools/mach-types添加一行"jz2440			MACH_JZ2440		JZ2440			1995"



==*阶段成果：kernel启动成功，在文件系统出挂死。*==

------

调试记录：

1.机器码不匹配，添加机器码MACH_TYPE_JZ2440 = 1995。

2.系统挂死，没有打印，修改bootargs中的console=ttySAC0,115200,主要是波特率之前没有，通过打开串口调试的宏发现波特率为0，修改后正常输出。

------

三、自制文件系统

1.按照网上教程操作即可

2.需要将编译链下面的库拷贝到lib目录下

3.可以从别处拷贝/etc目录的内容放在/etc下面



==*阶段成果：文件系统启动成功。*==

------

调试记录：

1.报错：exitcode=0x00000004，使用readelf -A vmlinux查看CPU架构是V4T,而查看busybox显示是V5TE。               

解决方法：

<1>make menuconfig 中指定架构 (-mcpu=arm920t) Additional CFLAGS  

<2>任然同样的错，确定架构没有问题了，通过上网查找，发现有人是拷贝的armv4t中的库，故重新拷贝库文件，顺利启动。



四、移植RTC

1.当前版本的内核，rtc-s3c.c只支持设备树，所以直接编译是匹配不到RTC设备的。

2.将老版本的linux内核中的rtc-s3c.c拷贝过来，替换掉即可。


