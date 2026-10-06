foreach s: [16, 24, 32, 48, 64, 128, 256, 512]
  s = '@0@x@0@'.format(s.to_string())
  install_data('qemu_@0@.png'.format(s),
               rename: 'qemu.png',
               install_dir: qemu_icondir / 'hicolor' / s / 'apps')
endforeach

install_data('qemu_32x32.bmp',
             rename: 'qemu.bmp',
             install_dir: qemu_icondir / 'hicolor' / '32x32' / 'apps')

# Spacebox: the product logo for the SDL window (ui/sdl2.c). Made from
# dev/spacebox/assets/brand/spacebox-logo-v1-white-1024.png by dev/spacebox/build/make-icon.sh
install_data('spacebox_512x512.bmp',
             rename: 'spacebox.bmp',
             install_dir: qemu_icondir / 'hicolor' / '512x512' / 'apps')

install_data('qemu.svg',
             install_dir: qemu_icondir / 'hicolor' / 'scalable' / 'apps')
