"""Wayland 使用系统 ScreenCast portal；X11 使用 ximagesrc。"""
import os
import uuid


class ScreenCapture:
    def __init__(self):
        self.connection = None
        self.session = None
        self.fd = None

    def source(self):
        if not os.environ.get('WAYLAND_DISPLAY'):
            return ['ximagesrc', 'use-damage=false', 'show-pointer=true'], ()
        import gi
        gi.require_version('Gio', '2.0')
        from gi.repository import Gio, GLib
        self.Gio, self.GLib = Gio, GLib
        self.connection = Gio.bus_get_sync(Gio.BusType.SESSION, None)
        result = self.request('CreateSession', '(a{sv})', (),
                              {'session_handle_token': GLib.Variant('s', 'lesson02_' + uuid.uuid4().hex)})
        self.session = result['session_handle']
        self.request('SelectSources', '(oa{sv})', (self.session,),
                     {'types': GLib.Variant('u', 1), 'multiple': GLib.Variant('b', False)})
        print('宝贝，请在系统录屏选择框中选择整个屏幕，以同时录到仿真器、相机和终端。', flush=True)
        result = self.request('Start', '(osa{sv})', (self.session, ''), {})
        streams = result.get('streams', [])
        if len(streams) != 1:
            raise RuntimeError('系统没有返回单个屏幕录制流')
        reply, fds = self.connection.call_with_unix_fd_list_sync(
            'org.freedesktop.portal.Desktop', '/org/freedesktop/portal/desktop',
            'org.freedesktop.portal.ScreenCast', 'OpenPipeWireRemote',
            GLib.Variant('(oa{sv})', (self.session, {})), GLib.VariantType.new('(h)'),
            Gio.DBusCallFlags.NONE, 10000, None, None)
        self.fd = fds.get(reply.unpack()[0])
        return ['pipewiresrc', 'fd=' + str(self.fd), 'path=' + str(streams[0][0]),
                'do-timestamp=true'], (self.fd,)

    def request(self, method, signature, prefix, options):
        Gio, GLib = self.Gio, self.GLib
        token = 'lesson02_' + uuid.uuid4().hex
        options = dict(options, handle_token=GLib.Variant('s', token))
        sender = self.connection.get_unique_name().lstrip(':').replace('.', '_')
        state = {'path': '/org/freedesktop/portal/desktop/request/' + sender + '/' + token}
        loop = GLib.MainLoop()

        def response(connection, sender_name, path, interface, signal_name, parameters, user_data):
            if path == state['path']:
                state['response'] = parameters.unpack()
                loop.quit()

        subscription = self.connection.signal_subscribe('org.freedesktop.portal.Desktop',
            'org.freedesktop.portal.Request', 'Response', None, None,
            Gio.DBusSignalFlags.NONE, response, None)
        timer = None
        try:
            reply = self.connection.call_sync('org.freedesktop.portal.Desktop',
                '/org/freedesktop/portal/desktop', 'org.freedesktop.portal.ScreenCast', method,
                GLib.Variant(signature, (*prefix, options)), GLib.VariantType.new('(o)'),
                Gio.DBusCallFlags.NONE, 10000, None)
            state['path'] = reply.unpack()[0]

            def timed_out():
                state['timeout'] = True
                loop.quit()
                return False
            timer = GLib.timeout_add_seconds(180, timed_out)
            loop.run()
        finally:
            if timer is not None and not state.get('timeout'):
                GLib.source_remove(timer)
            self.connection.signal_unsubscribe(subscription)
        if 'response' not in state:
            raise RuntimeError('系统录屏选择等待超时')
        code, result = state['response']
        if code != 0:
            raise RuntimeError('系统录屏已取消或创建失败，响应码=' + str(code))
        return result

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        if self.connection is not None and self.session is not None:
            self.connection.call_sync('org.freedesktop.portal.Desktop', self.session,
                'org.freedesktop.portal.Session', 'Close', None, None,
                self.Gio.DBusCallFlags.NONE, 5000, None)
            self.session = None
